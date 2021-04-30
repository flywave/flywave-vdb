#include "Version.hpp"
#include "pbr_renderer.hh"

#include "thread/ThreadUtils.hpp"

#include "io/CliParser.hpp"
#include "io/ImageIO.hpp"
#include "io/JsonLoadException.hpp"
#include "io/Scene.hpp"

#include "denoiser/NlMeans.hpp"
#include "denoiser/Pixmap.hpp"
#include "denoiser/Regression.hpp"

#include "Logging.hpp"
#include "Memory.hpp"
#include "Timer.hpp"

#include <cstdlib>
#include <cstring>
#include <tinyformat/tinyformat.hpp>

namespace flywave {

using namespace Tungsten;

template <typename Texel> struct render_buffer {
  std::unique_ptr<Pixmap<Texel>> buffer;
  std::unique_ptr<Pixmap<Texel>> bufferA;
  std::unique_ptr<Pixmap<Texel>> bufferB;
  std::unique_ptr<Pixmap<Texel>> bufferVariance;
};
typedef render_buffer<float> render_buffer_f;
typedef render_buffer<Vec3f> render_buffer_3f;

Pixmap3f nfor_denoiser(render_buffer_3f image,
                       std::vector<render_buffer_f> features) {
  int w = image.buffer->w(), h = image.buffer->h();

  std::vector<PixmapF> filteredFeaturesA(features.size());
  std::vector<PixmapF> filteredFeaturesB(features.size());
  SimdNlMeans featureFilter;
  for (size_t i = 0; i < features.size(); ++i) {
    featureFilter.addBuffer(filteredFeaturesA[i], *features[i].bufferA,
                            *features[i].bufferB, *features[i].bufferVariance);
    featureFilter.addBuffer(filteredFeaturesB[i], *features[i].bufferB,
                            *features[i].bufferA, *features[i].bufferVariance);
  }
  featureFilter.denoise(3, 5, 0.5f, 2.0f);
  features.clear();

  std::vector<Pixmap3f> filteredColorsA;
  std::vector<Pixmap3f> filteredColorsB;
  std::vector<Pixmap3f> mses;
  for (float k : {0.5f, 1.0f}) {
    Pixmap3f filteredColorA = collaborativeRegression(
        *image.bufferA, *image.bufferB, filteredFeaturesB,
        *image.bufferVariance, 3, 9, k);
    Pixmap3f filteredColorB = collaborativeRegression(
        *image.bufferB, *image.bufferA, filteredFeaturesA,
        *image.bufferVariance, 3, 9, k);

    // MSE estimation (section 5.3)
    Pixmap3f noisyMse(w, h);
    for (int i = 0; i < w * h; ++i) {
      Vec3f mseA = sqr((*image.bufferB)[i] - filteredColorA[i]) -
                   2.0f * (*image.bufferVariance)[i];
      Vec3f mseB = sqr((*image.bufferA)[i] - filteredColorB[i]) -
                   2.0f * (*image.bufferVariance)[i];
      Vec3f residualColorVariance =
          sqr(filteredColorB[i] - filteredColorA[i]) * 0.25f;

      noisyMse[i] = (mseA + mseB) * 0.5f - residualColorVariance;
    }
    filteredColorsA.emplace_back(std::move(filteredColorA));
    filteredColorsB.emplace_back(std::move(filteredColorB));

    // MSE filtering
    mses.emplace_back(nlMeans(noisyMse, *image.buffer, *image.bufferVariance, 1,
                              9, 1.0f, 1.0f, true));
  }

  // Bandwidth selection (section 5.3)
  // Generate selection map
  Pixmap3f noisySelection(w, h);
  for (int i = 0; i < w * h; ++i)
    for (int j = 0; j < 3; ++j)
      noisySelection[i][j] = mses[0][i][j] < mses[1][i][j] ? 0.0f : 1.0f;
  mses.clear();
  // Filter selection map
  Pixmap3f selection = nlMeans(noisySelection, *image.buffer,
                               *image.bufferVariance, 1, 9, 1.0f, 1.0f, true);

  // Apply selection map
  Pixmap3f resultA(w, h);
  Pixmap3f resultB(w, h);
  for (int i = 0; i < w * h; ++i) {
    resultA[i] +=
        lerp(filteredColorsA[0][i], filteredColorsA[1][i], selection[i]);
    resultB[i] +=
        lerp(filteredColorsB[0][i], filteredColorsB[1][i], selection[i]);
  }
  selection.reset();
  filteredColorsA.clear();
  filteredColorsB.clear();

  // Second filter pass (section 5.4)
  std::vector<PixmapF> finalFeatures;
  for (size_t i = 0; i < filteredFeaturesA.size(); ++i) {
    PixmapF combinedFeature(w, h);
    PixmapF combinedFeatureVar(w, h);

    for (int j = 0; j < w * h; ++j) {
      combinedFeature[j] =
          (filteredFeaturesA[i][j] + filteredFeaturesB[i][j]) * 0.5f;
      combinedFeatureVar[j] =
          sqr(filteredFeaturesB[i][j] - filteredFeaturesA[i][j]) * 0.25f;
    }
    filteredFeaturesA[i].reset();
    filteredFeaturesB[i].reset();

    finalFeatures.emplace_back(nlMeans(combinedFeature, combinedFeature,
                                       combinedFeatureVar, 3, 2, 0.5f));
  }

  Pixmap3f combinedResult(w, h);
  Pixmap3f combinedResultVar(w, h);
  for (int j = 0; j < w * h; ++j) {
    combinedResult[j] = (resultA[j] + resultB[j]) * 0.5f;
    combinedResultVar[j] = sqr(resultB[j] - resultA[j]) * 0.25f;
  }
  return collaborativeRegression(combinedResult, combinedResult, finalFeatures,
                                 combinedResultVar, 3, 9, 1.0f);
}

std::unique_ptr<PixmapF> slice_pixmap(const Pixmap3f &src, int channel) {
  int w = src.w(), h = src.h();

  auto result = std::unique_ptr<PixmapF>(new PixmapF(w, h));
  for (int j = 0; j < w * h; ++j)
    (*result)[j] = src[j][channel];

  return std::move(result);
}

void load_input_buffers(render_buffer_3f &image,
                        std::vector<render_buffer_f> &features,
                        const Scene &scene) {
  for (const auto &b : scene.rendererSettings().renderOutputs()) {
    if (!b.hdrOutputFile().empty()) {
      Path file = b.hdrOutputFile();
      auto buffer = loadPixmap<Vec3f>(file, true);
      if (buffer) {
        std::unique_ptr<Pixmap3f> bufferVariance;
        if (b.sampleVariance()) {
          Path varianceFile =
              file.stripExtension() + "Variance" + file.extension();
          bufferVariance = loadPixmap<Vec3f>(varianceFile);
        }
        std::unique_ptr<Pixmap3f> bufferA, bufferB;
        if (b.twoBufferVariance()) {
          Path fileA = file.stripExtension() + "A" + file.extension();
          Path fileB = file.stripExtension() + "B" + file.extension();
          bufferA = loadPixmap<Vec3f>(fileA);
          bufferB = loadPixmap<Vec3f>(fileB);
        }

        if (b.type() == OutputColor) {
          image.buffer = std::move(buffer);
          image.bufferA = std::move(bufferA);
          image.bufferB = std::move(bufferB);
          image.bufferVariance = std::move(bufferVariance);
        } else {
          bool isRgb = (b.type() == OutputNormal || b.type() == OutputAlbedo);
          for (int i = 0; i < (isRgb ? 3 : 1); ++i) {
            features.emplace_back();
            if (buffer)
              features.back().buffer = slice_pixmap(*buffer, i);
            if (bufferA)
              features.back().bufferA = slice_pixmap(*bufferA, i);
            if (bufferB)
              features.back().bufferB = slice_pixmap(*bufferB, i);
            if (bufferVariance)
              features.back().bufferVariance = slice_pixmap(*bufferVariance, i);
          }
        }
      }
    }
  }
}

bool pbr_renderer::denoiser(const std::string &target) {
  Tungsten::Path targetFile(target);

  Tungsten::ThreadUtils::startThreads(
      std::max(Tungsten::ThreadUtils::idealThreadCount() - 1, 1u));

  render_buffer_3f image;
  std::vector<render_buffer_f> features;
  load_input_buffers(image, features, *_scene);

  Tungsten::Timer timer;
  Tungsten::Pixmap3f result =
      nfor_denoiser(std::move(image), std::move(features));
  timer.stop();

  result.save(targetFile, true);

  return true;
}

} // namespace flywave
