
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#pragma once
#endif

#ifndef PBRT_FILM_IMAGE_H
#define PBRT_FILM_IMAGE_H

// film/image.h*
#include "pbrt.h"
#include "film.h"
#include "sampler.h"
#include "filter.h"
#include "paramset.h"

// ImageFilm Declarations
class ImageFilm : public Film {
public:
    // ImageFilm Public Methods
    ImageFilm(int xres, int yres, Filter *filt, const float crop[4],
              const string &filename, bool openWindow);
    ~ImageFilm() {
        delete pixels;
        delete filter;
        delete[] filterTable;
    }
    void AddSample(const CameraSample &sample, const Spectrum &L);
    void Splat(const CameraSample &sample, const Spectrum &L);
    void GetSampleExtent(int *xstart, int *xend, int *ystart, int *yend) const;
    void GetPixelExtent(int *xstart, int *xend, int *ystart, int *yend) const;
    void WriteImage(float splatScale);
    void UpdateDisplay(int x0, int y0, int x1, int y1, float splatScale);
private:
    // ImageFilm Private Data
    // 每一个采样点会影响到周围若干点的像素值，影响的权重由filter决定
    // filter包含均值滤波，高斯滤波等
    Filter *filter;
    // [0, 1]，用于裁剪渲染窗口，以实现多设备同时渲染？
    float cropWindow[4];
    string filename;
    int xPixelStart, yPixelStart, xPixelCount, yPixelCount;
    struct Pixel {
        Pixel() {
            for (int i = 0; i < 3; ++i) Lxyz[i] = splatXYZ[i] = 0.f;
            weightSum = 0.f;
        }
        float Lxyz[3];
        float weightSum;
        float splatXYZ[3];
        // 保证一个像素的数据正好占据一个cache，防止取像素值时导致多个cache Miss
        // 一个cache占32B，加上pad正好一个Pixel对象占据一个cache
        float pad;
    };
    // 像素采样时需要周围四个方向的像素值
    // 以二维的块的结构存储Pixel，减少像素值采样时的Miss次数
    BlockedArray<Pixel> *pixels;
    float *filterTable;
};


ImageFilm *CreateImageFilm(const ParamSet &params, Filter *filter);

#endif // PBRT_FILM_IMAGE_H
