#include "pbr_renderer.hh"
#include <lodepng/lodepng.h>

#include <io/FileIterables.hpp>
#include <io/JsonLoadException.hpp>
#include <io/Path.hpp>
#include <io/Scene.hpp>
#include <io/ZipWriter.hpp>

#include <iostream>
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <unordered_map>
#include <unordered_set>

namespace flywave {

const char *render_state_to_string(render_state state) {
  switch (state) {
  case STATE_LOADING:
    return "loading";
  case STATE_RENDERING:
    return "rendering";
  default:
    return "unknown";
  }
}

void pbr_renderer::set_checkpoint_interval(double inter) {
  _checkpointInterval = inter;
}

void pbr_renderer::set_timeout(double t) { _timeout = t; }

void pbr_renderer::set_thread_count(int threadCount) {
  _threadCount = threadCount;
}

void pbr_renderer::set_input_directory(const std::string &path) {
  _inputDirectory = Tungsten::Path(path);
}

void pbr_renderer::set_output_directory(const std::string &path) {
  _outputDirectory = Tungsten::Path(path);
}

bool pbr_renderer::render_scene() {
  Tungsten::Path currentScene;
  {
    std::unique_lock<std::mutex> lock(_statusMutex);
    if (_status.queuedScenes.empty())
      return false;

    _status.state = STATE_LOADING;
    _status.startSpp = _status.currentSpp = _status.nextSpp = _status.totalSpp =
        0;

    currentScene = _status.currentScene = _status.queuedScenes.front();
    _status.queuedScenes.pop_front();
  }

  Tungsten::Path inputDirectory = _inputDirectory;
  try {
    std::unique_lock<std::mutex> lock(_sceneMutex);
    _scene.reset(Tungsten::Scene::load(Tungsten::Path(currentScene), nullptr,
                                       &inputDirectory));
    _scene->loadResources();
  } catch (const Tungsten::JsonLoadException &e) {
    std::cerr << e.what() << std::endl;

    std::unique_lock<std::mutex> lock(_sceneMutex);
    _scene.reset();

    return true;
  }

  {
    std::unique_lock<std::mutex> lock(_statusMutex);
    _status.totalSpp = _scene->rendererSettings().spp();
  }

  try {
    Tungsten::DirectoryChange context(inputDirectory);
    _scene->rendererSettings().setOutputDirectory(_outputDirectory);

    {
      std::unique_lock<std::mutex> lock(_sceneMutex);
      _flattenedScene.reset(_scene->makeTraceable(_seed));
    }
    Tungsten::Integrator &integrator = _flattenedScene->integrator();
    bool resumeRender = _scene->rendererSettings().enableResumeRender();
    if (resumeRender && !integrator.supportsResumeRender()) {
      resumeRender = false;
    }

    Tungsten::Timer timer, checkpointTimer;
    double totalElapsed = 0.0;
    while (!integrator.done()) {
      {
        std::unique_lock<std::mutex> lock(_statusMutex);
        _status.state = STATE_RENDERING;
        _status.currentSpp = integrator.currentSpp();
        _status.nextSpp = integrator.nextSpp();
      }

      integrator.startRender([]() {});
      integrator.waitForCompletion();

      timer.stop();
      if (_timeout > 0.0 && timer.elapsed() > _timeout)
        break;
      checkpointTimer.stop();
      if (_checkpointInterval > 0.0 &&
          checkpointTimer.elapsed() > _checkpointInterval) {
        totalElapsed += checkpointTimer.elapsed();
        Tungsten::Timer ioTimer;
        checkpointTimer.start();
        integrator.saveCheckpoint();
        if (resumeRender)
          integrator.saveRenderResumeData(*_scene);
        ioTimer.stop();
      }
    }
    timer.stop();

    integrator.saveOutputs();
    if (_scene->rendererSettings().enableResumeRender())
      integrator.saveRenderResumeData(*_scene);

    {
      std::unique_lock<std::mutex> lock(_statusMutex);
      _status.completedScenes.push_back(currentScene);
    }
  } catch (std::runtime_error &e) {
    return false;
  }

  {
    std::unique_lock<std::mutex> lock(_sceneMutex);
    _flattenedScene.reset();
    _scene.reset();
  }

  return true;
}

void pbr_renderer::setup() {
  Tungsten::EmbreeUtil::initDevice();
  openvdb::initialize();
  Tungsten::ThreadUtils::startThreads(_threadCount);
}

renderer_status pbr_renderer::status() {
  std::unique_lock<std::mutex> lock(_statusMutex);
  renderer_status copy(_status);
  return std::move(copy);
}

std::unique_ptr<Tungsten::Vec3c[]>
pbr_renderer::frame_buffer(Tungsten::Vec2i &resolution) {
  std::unique_lock<std::mutex> lock(_sceneMutex);
  if (!_scene || !_flattenedScene)
    return nullptr;

  Tungsten::Vec2u res = _scene->camera()->resolution();
  std::unique_ptr<Tungsten::Vec3c[]> ldr(new Tungsten::Vec3c[res.product()]);

  for (Tungsten::uint32 y = 0; y < res.y(); ++y)
    for (Tungsten::uint32 x = 0; x < res.x(); ++x)
      ldr[x + y * res.x()] = Tungsten::Vec3c(
          Tungsten::clamp(Tungsten::Vec3i(_scene->camera()->get(x, y) * 255.0f),
                          Tungsten::Vec3i(0), Tungsten::Vec3i(255)));

  resolution = Tungsten::Vec2i(res);

  return std::move(ldr);
}

std::unique_ptr<Tungsten::uint8, void (*)(void *)>
pbr_renderer::frame_buffer_png(Tungsten::Vec2i &resolution) {
  std::unique_ptr<Tungsten::uint8, void (*)(void *)> null_(
      nullptr, [](void *) -> void {});
  std::unique_ptr<Tungsten::Vec3c[]> ldr = frame_buffer(resolution);
  if (!ldr)
    return std::move(null_);

  Tungsten::uint8 *encoded = nullptr;
  size_t encodedSize;
  if (lodepng_encode_memory(&encoded, &encodedSize, &ldr[0].x(), resolution.x(),
                            resolution.y(), LCT_RGB, 8) != 0)
    return std::move(null_);

  ldr.reset();

  return std::unique_ptr<Tungsten::uint8, void (*)(void *)>(encoded, free);
}

inline bool zip_resources(Tungsten::Scene *scene, const std::string &output,
                          int compressionLevel = 5) {
  std::unordered_set<Tungsten::Path> remappedPaths;
  auto remap = [&](const Tungsten::Path &p) {
    Tungsten::Path result = p.stripParent();
    int index = 1;
    if (p.isDirectory())
      while (remappedPaths.count(result))
        result = p.baseName() + tfm::format("%03d", index++);
    else
      while (remappedPaths.count(result))
        result = p.baseName() + tfm::format("%03d", index++) + p.extension();
    remappedPaths.insert(result);
    return (result);
  };

  try {
    Tungsten::Path outpath(output);
    Tungsten::ZipWriter writer(outpath);
    auto addFile = [&](const Tungsten::Path &src, const Tungsten::Path &dst) {
      if (!writer.addFile(src, dst, compressionLevel))
        std::cout << "Warning: Failed to add file " << src << " to zip package"
                  << std::endl;
    };
    auto addDirectory = [&](const Tungsten::Path &dst) {
      if (!writer.addDirectory(dst))
        std::cout << "Warning: Failed to add directory " << dst
                  << " to zip package" << std::endl;
    };

    for (const auto &r : scene->resources()) {
      Tungsten::Path &path = *r.second;
      Tungsten::Path zipPath = remap(path);

      if (path.isDirectory()) {
        Tungsten::Path root(path, "");
        for (const Tungsten::Path &p : root.recursive()) {
          if (p.isDirectory())
            addDirectory(zipPath / p);
          else
            addFile(p, zipPath / p);
        }
      } else {
        addFile(path, zipPath);
      }

      path = zipPath;
    }

    rapidjson::Document document;
    document.SetObject();
    *(static_cast<rapidjson::Value *>(&document)) =
        scene->toJson(document.GetAllocator());

    rapidjson::GenericStringBuffer<rapidjson::UTF8<>> buffer;
    rapidjson::PrettyWriter<rapidjson::GenericStringBuffer<rapidjson::UTF8<>>>
        jsonWriter(buffer);
    document.Accept(jsonWriter);

    std::string json = buffer.GetString();
    writer.addFile(json.c_str(), json.size(), scene->path().fileName(),
                   compressionLevel);
  } catch (const std::runtime_error &e) {
    return false;
  }
  return true;
}

inline bool relocate_resources(Tungsten::Scene *scene,
                               const std::string &output_s,
                               bool copyRelocate = false) {
  Tungsten::Path output(output_s);
  if (!Tungsten::FileUtils::createDirectory(output, true))
    return false;

  Tungsten::Path resourceParent = output;

  Tungsten::Path normalizedOutput = output.normalize();
  Tungsten::Path outputTail;
  Tungsten::Path normalizedSceneFolder = scene->path().parent().normalize();
  while (!normalizedOutput.empty()) {
    if (normalizedOutput == normalizedSceneFolder) {
      resourceParent = outputTail;
      break;
    }
    outputTail = normalizedOutput.fileName() / outputTail;
    normalizedOutput = normalizedOutput.parent().stripSeparator();
  }

  for (const auto &r : scene->resources()) {
    Tungsten::Path newPath = output / r.first.fileName();

    bool success;
    if (copyRelocate)
      success = Tungsten::FileUtils::copyFile(r.first, newPath, false);
    else
      success = Tungsten::FileUtils::moveFile(r.first, newPath, true);

    if (!success)
      return success;
    else
      *r.second = resourceParent / r.first.fileName();
  }

  Tungsten::Scene::save(scene->path(), *scene);
  return true;
}

bool pbr_renderer::relocate(const std::string &output_s, bool copyRelocate) {
  return relocate_resources(_scene.get(), output_s, copyRelocate);
}

bool pbr_renderer::ziparchive(const std::string &output, int compressionLevel) {
  return zip_resources(_scene.get(), output, compressionLevel);
}

} // namespace flywave
