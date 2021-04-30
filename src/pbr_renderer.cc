#include "pbr_renderer.hh"

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
} // namespace flywave
