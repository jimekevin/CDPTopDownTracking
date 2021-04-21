// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include "example-imgui.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <chrono>

#include "RealsenseCameraManager.h"

void render_slider(rect location, float& clipping_dist);
void remove_background(rs2::video_frame& other, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist);
float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

int main(int argc, char * argv[]) try
{
	// Create and initialize GUI related objects
	window app(1280, 720, "RealSense Align (Advanced) Example"); // Simple window handling
	ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
	rs2::colorizer c;                     // Helper to colorize depth images
	texture renderer;                     // Helper for renderig images

	auto rcm = new RealsenseCameraManager();

	std::chrono::steady_clock::time_point fpsLast;
	long long fps;
	while (app) // Application still alive?
	{

		fpsLast = std::chrono::high_resolution_clock::now();

		rcm->pollFrames();

		if (!rcm->processFrames()) {
			continue;
		}

		// Taking dimensions of the window for rendering purposes
		float w = static_cast<float>(app.width());
		float h = static_cast<float>(app.height());

#if RENDER_RS2
		rs2::video_frame other_frame = rcm->getRs2ColorFrame();
#else
		cv::Mat other_frame = rcm->getCvColorFrame();
		//cv::Mat aligned_depth_frame = rcm->getCvDepthFrame();
#endif
		rs2::depth_frame aligned_depth_frame = rcm->getRs2DepthFrame();

		// At this point, "other_frame" is an altered frame, stripped form its background
		// Calculating the position to place the frame in the window
		rect altered_other_frame_rect{ 0, 0, w, h };
#if RENDER_RS2
		altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.get_width()),static_cast<float>(other_frame.get_height()) });
#else
		altered_other_frame_rect = altered_other_frame_rect.adjust_ratio({ static_cast<float>(other_frame.cols),static_cast<float>(other_frame.rows) });
#endif

		// Render aligned image
		renderer.render(other_frame, altered_other_frame_rect);

		// The example also renders the depth frame, as a picture-in-picture
		// Calculating the position to place the depth frame in the window
		rect pip_stream{ 0, 0, w / 5, h / 5 };
		pip_stream = pip_stream.adjust_ratio({ static_cast<float>(aligned_depth_frame.get_width()),static_cast<float>(aligned_depth_frame.get_height()) });
		pip_stream.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream.w - (std::max(w, h) / 25);
		pip_stream.y = altered_other_frame_rect.y + (std::max(w, h) / 25);

		// Render depth (as picture in pipcture)
		renderer.upload(c.process(aligned_depth_frame));
		renderer.show(pip_stream);

		// Using ImGui library to provide a slide controller to select the depth clipping distance
		ImGui_ImplGlfw_NewFrame(1);
		float depth_clipping_distance = 2.f;
		render_slider({ 5.f, 0, w, h }, depth_clipping_distance);
		ImGui::Render();

		// FPS
		auto fpsNow = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(fpsNow - fpsLast).count();
		if (duration > 0) {
			fps = CLOCKS_PER_SEC / duration;
		}
		std::cout << "FPS: " << fps << "\r";

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

float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

void render_slider(rect location, float& clipping_dist)
{
	// Some trickery to display the control nicely
	static const int flags = ImGuiWindowFlags_NoCollapse
		| ImGuiWindowFlags_NoScrollbar
		| ImGuiWindowFlags_NoSavedSettings
		| ImGuiWindowFlags_NoTitleBar
		| ImGuiWindowFlags_NoResize
		| ImGuiWindowFlags_NoMove;
	const int pixels_to_buttom_of_stream_text = 25;
	const float slider_window_width = 30;

	ImGui::SetNextWindowPos({ location.x, location.y + pixels_to_buttom_of_stream_text });
	ImGui::SetNextWindowSize({ slider_window_width + 20, location.h - (pixels_to_buttom_of_stream_text * 2) });

	//Render the vertical slider
	ImGui::Begin("slider", nullptr, flags);
	ImGui::PushStyleColor(ImGuiCol_FrameBg, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
	ImGui::PushStyleColor(ImGuiCol_SliderGrab, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
	ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, ImColor(215.f / 255, 215.0f / 255, 215.0f / 255));
	auto slider_size = ImVec2(slider_window_width / 2, location.h - (pixels_to_buttom_of_stream_text * 2) - 20);
	ImGui::VSliderFloat("", slider_size, &clipping_dist, 0.0f, 6.0f, "", 1.0f, true);
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("Depth Clipping Distance: %.3f", clipping_dist);
	ImGui::PopStyleColor(3);

	//Display bars next to slider
	float bars_dist = (slider_size.y / 6.0f);
	for (int i = 0; i <= 6; i++)
	{
		ImGui::SetCursorPos({ slider_size.x, i * bars_dist });
		std::string bar_text = "- " + std::to_string(6 - i) + "m";
		ImGui::Text("%s", bar_text.c_str());
	}
	ImGui::End();
}

void remove_background(rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist)
{
	const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
	uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

	int width = other_frame.get_width();
	int height = other_frame.get_height();
	int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
	for (int y = 0; y < height; y++)
	{
		auto depth_pixel_index = y * width;
		for (int x = 0; x < width; x++, ++depth_pixel_index)
		{
			// Get the depth value of the current pixel
			auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

			// Check if the depth value is invalid (<=0) or greater than the threashold
			if (pixels_distance <= 0.f || pixels_distance > clipping_dist)
			{
				// Calculate the offset in other frame's buffer to current pixel
				auto offset = depth_pixel_index * other_bpp;

				// Set pixel to "background" color (0x999999)
				std::memset(&p_other_frame[offset], 0x99, other_bpp);
			}
		}
	}
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
	//Given a vector of streams, we try to find a depth stream and another stream to align depth with.
	//We prioritize color streams to make the view look better.
	//If color is not available, we take another stream that (other than depth)
	rs2_stream align_to = RS2_STREAM_ANY;
	bool depth_stream_found = false;
	bool color_stream_found = false;
	for (rs2::stream_profile sp : streams)
	{
		rs2_stream profile_stream = sp.stream_type();
		if (profile_stream != RS2_STREAM_DEPTH)
		{
			if (!color_stream_found)         //Prefer color
				align_to = profile_stream;

			if (profile_stream == RS2_STREAM_COLOR)
			{
				color_stream_found = true;
			}
		}
		else
		{
			depth_stream_found = true;
		}
	}

	if (!depth_stream_found)
		throw std::runtime_error("No Depth stream available");

	if (align_to == RS2_STREAM_ANY)
		throw std::runtime_error("No stream found to align with Depth");

	return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}
