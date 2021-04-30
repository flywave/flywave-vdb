#pragma once

#include <primitives/EmbreeUtil.hpp>

#include <renderer/TraceableScene.hpp>

#include <thread/ThreadUtils.hpp>

#include <io/CliParser.hpp>
#include <io/DirectoryChange.hpp>
#include <io/FileUtils.hpp>
#include <io/JsonLoadException.hpp>
#include <io/JsonObject.hpp>
#include <io/Scene.hpp>
#include <io/StringUtils.hpp>

#include <Timer.hpp>

#include <cstdlib>
#include <deque>
#include <mutex>
#include <rapidjson/document.h>
#include <tinyformat/tinyformat.hpp>
#include <vector>

#include <openvdb/openvdb.h>

namespace flywave {

enum render_state {
  STATE_LOADING,
  STATE_RENDERING,
};

const char *render_state_to_string(render_state state);

struct renderer_status {
  render_state state;
  int startSpp;
  int currentSpp;
  int nextSpp;
  int totalSpp;

  std::vector<Tungsten::Path> completedScenes;
  Tungsten::Path currentScene;
  std::deque<Tungsten::Path> queuedScenes;

  rapidjson::Value toJson(rapidjson::Document::AllocatorType &allocator) const {
    Tungsten::JsonObject result{
        allocator,   "state",    render_state_to_string(state),
        "start_spp", startSpp,   "current_spp",
        currentSpp,  "next_spp", nextSpp,
        "total_spp", totalSpp,   "current_scene",
        currentScene};
    rapidjson::Value completedValue(rapidjson::kArrayType);
    rapidjson::Value queuedValue(rapidjson::kArrayType);

    for (const Tungsten::Path &p : completedScenes)
      completedValue.PushBack(Tungsten::JsonUtils::toJson(p, allocator),
                              allocator);
    for (const Tungsten::Path &p : queuedScenes)
      queuedValue.PushBack(Tungsten::JsonUtils::toJson(p, allocator),
                           allocator);

    result.add("completed_scenes", std::move(completedValue), "queued_scenes",
               std::move(queuedValue));

    return result;
  }
};

class pbr_renderer {
  double _checkpointInterval;
  double _timeout;
  int _threadCount;
  Tungsten::Path _inputDirectory;
  Tungsten::Path _outputDirectory;

  std::unique_ptr<Tungsten::Scene> _scene;
  std::unique_ptr<Tungsten::TraceableScene> _flattenedScene;

  std::mutex _statusMutex;
  std::mutex _sceneMutex;
  renderer_status _status;
  Tungsten::uint32 _seed;

public:
  pbr_renderer()
      : _checkpointInterval(0.0), _timeout(0.0),
        _threadCount(
            std::max(Tungsten::ThreadUtils::idealThreadCount() - 1, 1u)),
        _seed(0xBA5EBA11) {
    _status.state = STATE_LOADING;
    _status.currentSpp = _status.nextSpp = _status.totalSpp = 0;
  }

  void set_checkpoint_interval(double inter);
  void set_timeout(double t);
  void set_thread_count(int threadCount);

  void set_input_directory(const std::string &path);
  void set_output_directory(const std::string &path);

  void setup();

  bool render_scene();

  renderer_status status();

  std::unique_ptr<Tungsten::Vec3c[]> frame_buffer(Tungsten::Vec2i &resolution);

  std::unique_ptr<Tungsten::uint8, void (*)(void *)>
  frame_buffer_png(Tungsten::Vec2i &resolution);
};

} // namespace flywave
