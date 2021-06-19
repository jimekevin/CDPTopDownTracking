// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "example-imgui.hpp"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <chrono>

#include "RealsenseCameraManager.h"

#ifdef APPLE
#define RECORDING_BT_BS "/Volumes/Kevkon/20210421_193350_bt_bs.bag"
#define RECORDING_BT_WS "/Volumes/Kevkon/20210421_193310_bt_ws.bag" // Removed for now
#define RECORDING_WT_WS "/Volumes/Kevkon/20210421_193129_wt_ws.bag"
#define RECORDING_WT_BS "/Volumes/Kevkon/20210421_193054_wt_bs.bag" // Removed for now (30.05.2021)
#define RECORDING_TABLE_0 "/Volumes/Kevkon/20210617_165603.bag"
#define RECORDING_TABLE_1 "/Volumes/Kevkon/20210617_165633.bag"
#define RECORDING_TABLE_2 "/Volumes/Kevkon/20210617_165711.bag"
#define SCREENSHOT_PATH "/Users/lilith/Desktop/"
#else
#define RECORDING_BT_BS "C:\\Users\\CDP Research Group\\Desktop\\ExternalTools\\TopDownTracking\\data\\20210421_193350_bt_bs.bag"
#define RECORDING_BT_WS "C:\\Users\\CDP Research Group\\Desktop\\ExternalTools\\TopDownTracking\\data\\20210421_193310_bt_ws.bag" // Removed for now
#define RECORDING_WT_WS "C:\\Users\\CDP Research Group\\Desktop\\ExternalTools\\TopDownTracking\\data\\20210421_193129_wt_ws.bag"
#define RECORDING_WT_BS "C:\\Users\\CDP Research Group\\Desktop\\ExternalTools\\TopDownTracking\\data\\20210421_193054_wt_bs.bag" // Removed for now (30.05.2021)
#define RECORDING_TABLE_0 "C:\\Users\\CDP Research Group\\Desktop\\ExternalTools\\TopDownTracking\\data\\20210617_165603.bag"
#define RECORDING_TABLE_1 "C:\\Users\\CDP Research Group\\Desktop\\ExternalTools\\TopDownTracking\\data\\20210617_165633.bag"
#define RECORDING_TABLE_2 "C:\\Users\\CDP Research Group\\Desktop\\ExternalTools\\TopDownTracking\\data\\20210617_165711.bag"
#define SCREENSHOT_PATH "C:\\Users\\CDP Research Group\\Desktop\\"
#endif

enum RECORDING { NONE=-1, LIVE, WT_BS, WT_WS, BT_BS, BT_WS, TABLE_0, TABLE_1, TABLE_2, COUNT };
inline const std::string getRecording(int id) {
    auto rec = static_cast<RECORDING>(id);
    switch (rec) {
        case LIVE: return "LIVE";
        case WT_BS: return RECORDING_WT_BS;
        case WT_WS: return RECORDING_WT_WS;
        case BT_BS: return RECORDING_BT_BS;
        case BT_WS: return RECORDING_BT_WS;
        case TABLE_0: return RECORDING_TABLE_0;
        case TABLE_1: return RECORDING_TABLE_1;
        case TABLE_2: return RECORDING_TABLE_2;
        default:    return "";
    }
}

int activeRecording = static_cast<int>(RECORDING::TABLE_0);
int lastRecording = activeRecording;

int main(int, char**) try
{
	// Create and initialize GUI related objects
	window app(1280, 720, "Top Down Tracking - CVTests (Kevin Bein)"); // Simple window handling
	ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
	rs2::colorizer c;                     // Helper to colorize depth images
	texture renderer;                     // Helper for renderig images

    auto rcm = std::make_unique<RealsenseCameraManager>();
    if (!rcm->Init(getRecording(activeRecording))) {
      std::cout << "Could not initialize RealsenseCameraManager\n";
      return 1;
    }

    double fps;
	std::chrono::steady_clock::time_point fpsLast;
	while (app) // Application still alive?
	{
	    if (activeRecording != lastRecording) {
	        rcm = std::make_unique<RealsenseCameraManager>();
          rcm->Init(getRecording(activeRecording));
	        lastRecording = activeRecording;
	    }

		fpsLast = std::chrono::high_resolution_clock::now();

      rcm->PollFrames();

		if (!rcm->ProcessFrames()) {
			continue;
		}

		// Taking dimensions of the window for rendering purposes
		auto w = static_cast<float>(app.width());
		auto h = static_cast<float>(app.height());
#ifdef APPLE
        auto w_video = w * 2;
        auto h_video = h * 2;
#else
        auto w_video = w;
        auto h_video = h;
#endif

#if RENDER_RS2
		rs2::video_frame other_frame = rcm->getRs2ColorFrame();
#else
		cv::Mat other_frame = rcm->GetCvColorFrame();
		//cv::Mat aligned_depth_frame = rcm->getCvDepthFrame();
#endif
		rs2::depth_frame aligned_depth_frame = rcm->GetRs2DepthFrame();

		// At this point, "other_frame" is an altered frame, stripped form its background
		// Calculating the position to place the frame in the window
		rect altered_other_frame_rect{ 0, 0, w_video, h_video };
#if RENDER_RS2
		altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.get_width()),static_cast<float>(other_frame.get_height()) });
#else
		altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.cols),static_cast<float>(other_frame.rows) });
#endif

		// Render aligned image
		renderer.render(other_frame, altered_other_frame_rect);

		// The example also renders the depth frame, as a picture-in-picture
		// Calculating the position to place the depth frame in the window
		rect pip_stream{ 0, 0, w_video / 5, h_video / 5 };
		pip_stream = pip_stream.adjust_ratio({ static_cast<float>(aligned_depth_frame.get_width()),static_cast<float>(aligned_depth_frame.get_height()) });
		pip_stream.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream.w - (std::max(w_video, h_video) / 25);
		pip_stream.y = altered_other_frame_rect.y + (std::max(w_video, h_video) / 25);

		// Render depth (as picture in pipcture)
		renderer.upload(c.process(aligned_depth_frame));
		renderer.show(pip_stream);

		// Using ImGui library to provide a slide controller to select the depth clipping distance
		ImGui_ImplGlfw_NewFrame(1);

		// Print controls
        ImGui::Begin("Controls");

        auto fpsNow = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(fpsNow - fpsLast).count();
        if (duration > 0) {
            fps = CLOCKS_PER_SEC / duration;
        }
        std::stringstream fpsOut;
        fpsOut << "FPS: " << fps;
        ImGui::Text("%s", fpsOut.str().c_str());

        ImGui::Checkbox("Stop playback", &rcm->prop_stopped_frame);
        if (ImGui::Button("Re-Calibrate")) rcm->Recalibrate();
        ImGui::SliderInt("Video ID", &activeRecording, 0, RECORDING::COUNT - 1);
        ImGui::SliderInt("Frame Step", &rcm->prop_frame_step, 0, 7);
        //ImGui::Text(rcm->getFrameStepLabel());
        ImGui::SliderFloat("Z Culling Back", &rcm->prop_z_culling_back, -3.0f, 3.0f);
        ImGui::SliderFloat("Z Culling Front", &rcm->prop_z_culling_front, -3.0f, 3.0f);
        ImGui::SliderIntWithSteps("Gaussian Kernel", &rcm->prop_gaussian_kernel, 1, 23, 2, "%.0f");
        ImGui::SliderFloat("Threshold", &rcm->prop_threshold, 0.0f, 1.0f);
        ImGui::SliderFloat("Threshold Max", &rcm->prop_threshold_max, 0.0f, 255.0f);
        ImGui::SliderIntWithSteps("Morph kernel", &rcm->prop_gaussian_kernel, 1, 23, 2, "%.0f");
        auto screenshotFlag = rcm->prop_save_screenshot ? RealsenseCameraManager::SCREENSHOT_FLAGS::DISPLAY_SAVE : RealsenseCameraManager::SCREENSHOT_FLAGS::DISPLAY;
        if (ImGui::Button("S 0")) rcm->Screenshot(0, screenshotFlag, SCREENSHOT_PATH); ImGui::SameLine();
        if (ImGui::Button("S 1")) rcm->Screenshot(1, screenshotFlag, SCREENSHOT_PATH); ImGui::SameLine();
        if (ImGui::Button("S 2")) rcm->Screenshot(2, screenshotFlag, SCREENSHOT_PATH); ImGui::SameLine();
        if (ImGui::Button("S 3")) rcm->Screenshot(3, screenshotFlag, SCREENSHOT_PATH); ImGui::SameLine();
        if (ImGui::Button("S 4")) rcm->Screenshot(4, screenshotFlag, SCREENSHOT_PATH); ImGui::SameLine();
        if (ImGui::Button("S 5")) rcm->Screenshot(5, screenshotFlag, SCREENSHOT_PATH); ImGui::SameLine();
        ImGui::Checkbox("Save", &rcm->prop_save_screenshot);
        ImGui::End();

        ImGui::Render();
	}
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
