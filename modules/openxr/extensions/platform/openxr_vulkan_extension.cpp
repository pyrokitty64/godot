/**************************************************************************/
/*  openxr_vulkan_extension.cpp                                           */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "openxr_vulkan_extension.h"

#include "../../openxr_util.h"
#include "../openxr_fb_foveation_extension.h"

#include "core/os/os.h"
#include "core/string/print_string.h"
#include "servers/rendering/rendering_device.h"
#include "servers/rendering/rendering_server.h"

#ifdef WINDOWS_ENABLED
#include <windows.h>
#include <vulkan/vulkan_win32.h>
#endif

HashMap<String, bool *> OpenXRVulkanExtension::get_requested_extensions(XrVersion p_version) {
	HashMap<String, bool *> request_extensions;

	request_extensions[XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME] = nullptr; // must be available

	return request_extensions;
}

void OpenXRVulkanExtension::on_instance_created(const XrInstance p_instance) {
	ERR_FAIL_NULL(OpenXRAPI::get_singleton());

	// Obtain pointers to functions we're accessing here, they are (not yet) part of core.

	EXT_INIT_XR_FUNC(xrGetVulkanGraphicsRequirements2KHR);
	EXT_INIT_XR_FUNC(xrCreateVulkanInstanceKHR);
	EXT_INIT_XR_FUNC(xrGetVulkanGraphicsDevice2KHR);
	EXT_INIT_XR_FUNC(xrCreateVulkanDeviceKHR);
	EXT_INIT_XR_FUNC(xrEnumerateSwapchainImages);
}

bool OpenXRVulkanExtension::check_graphics_api_support(XrVersion p_desired_version) {
	ERR_FAIL_NULL_V(OpenXRAPI::get_singleton(), false);

	XrGraphicsRequirementsVulkan2KHR vulkan_requirements = {
		XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR, // type
		nullptr, // next
		0, // minApiVersionSupported
		0 // maxApiVersionSupported
	};

	XrResult result = xrGetVulkanGraphicsRequirements2KHR(OpenXRAPI::get_singleton()->get_instance(), OpenXRAPI::get_singleton()->get_system_id(), &vulkan_requirements);
	if (XR_FAILED(result)) {
		print_line("OpenXR: Failed to get Vulkan graphics requirements [", OpenXRAPI::get_singleton()->get_error_string(result), "]");
		return false;
	}

	// #ifdef DEBUG
	print_line("OpenXR: XrGraphicsRequirementsVulkan2KHR:");
	print_line(" - minApiVersionSupported: ", OpenXRUtil::make_xr_version_string(vulkan_requirements.minApiVersionSupported));
	print_line(" - maxApiVersionSupported: ", OpenXRUtil::make_xr_version_string(vulkan_requirements.maxApiVersionSupported));
	// #endif

	if (p_desired_version < vulkan_requirements.minApiVersionSupported) {
		print_line("OpenXR: Requested Vulkan version does not meet the minimum version this runtime supports.");
		print_line("- desired_version ", OpenXRUtil::make_xr_version_string(p_desired_version));
		print_line("- minApiVersionSupported ", OpenXRUtil::make_xr_version_string(vulkan_requirements.minApiVersionSupported));
		print_line("- maxApiVersionSupported ", OpenXRUtil::make_xr_version_string(vulkan_requirements.maxApiVersionSupported));
		return false;
	}

	if (p_desired_version > vulkan_requirements.maxApiVersionSupported) {
		print_line("OpenXR: Requested Vulkan version exceeds the maximum version this runtime has been tested on and is known to support.");
		print_line("- desired_version ", OpenXRUtil::make_xr_version_string(p_desired_version));
		print_line("- minApiVersionSupported ", OpenXRUtil::make_xr_version_string(vulkan_requirements.minApiVersionSupported));
		print_line("- maxApiVersionSupported ", OpenXRUtil::make_xr_version_string(vulkan_requirements.maxApiVersionSupported));
	}

	return true;
}

bool OpenXRVulkanExtension::create_vulkan_instance(const VkInstanceCreateInfo *p_vulkan_create_info, VkInstance *r_instance) {
	// get the vulkan version we are creating
	uint32_t vulkan_version = p_vulkan_create_info->pApplicationInfo->apiVersion;
	uint32_t major_version = VK_VERSION_MAJOR(vulkan_version);
	uint32_t minor_version = VK_VERSION_MINOR(vulkan_version);
	uint32_t patch_version = VK_VERSION_PATCH(vulkan_version);
	XrVersion desired_version = XR_MAKE_VERSION(major_version, minor_version, patch_version);

	// check if this is supported
	if (!check_graphics_api_support(desired_version)) {
		return false;
	}

	XrVulkanInstanceCreateInfoKHR xr_vulkan_instance_info = {
		XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR, // type
		nullptr, // next
		OpenXRAPI::get_singleton()->get_system_id(), // systemId
		0, // createFlags
		vkGetInstanceProcAddr, // pfnGetInstanceProcAddr
		p_vulkan_create_info, // vulkanCreateInfo
		nullptr, // vulkanAllocator
	};

	VkResult vk_result = VK_SUCCESS;
	XrResult result = xrCreateVulkanInstanceKHR(OpenXRAPI::get_singleton()->get_instance(), &xr_vulkan_instance_info, &vulkan_instance, &vk_result);
	if (XR_FAILED(result)) {
		print_line("OpenXR: Failed to create Vulkan instance [", OpenXRAPI::get_singleton()->get_error_string(result), "]");
		return false;
	}

	ERR_FAIL_COND_V_MSG(vk_result == VK_ERROR_INCOMPATIBLE_DRIVER, false,
			"Cannot find a compatible Vulkan installable client driver (ICD).\n\n"
			"vkCreateInstance Failure");
	ERR_FAIL_COND_V_MSG(vk_result == VK_ERROR_EXTENSION_NOT_PRESENT, false,
			"Cannot find a specified extension library.\n"
			"Make sure your layers path is set appropriately.\n"
			"vkCreateInstance Failure");
	ERR_FAIL_COND_V_MSG(vk_result, false,
			"vkCreateInstance failed.\n\n"
			"Do you have a compatible Vulkan installable client driver (ICD) installed?\n"
			"Please look at the Getting Started guide for additional information.\n"
			"vkCreateInstance Failure");

	*r_instance = vulkan_instance;

	return true;
}

bool OpenXRVulkanExtension::get_physical_device(VkPhysicalDevice *r_device) {
	ERR_FAIL_NULL_V(OpenXRAPI::get_singleton(), false);

	XrVulkanGraphicsDeviceGetInfoKHR get_info = {
		XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR, // type
		nullptr, // next
		OpenXRAPI::get_singleton()->get_system_id(), // systemId
		vulkan_instance, // vulkanInstance
	};

	XrResult result = xrGetVulkanGraphicsDevice2KHR(OpenXRAPI::get_singleton()->get_instance(), &get_info, &vulkan_physical_device);
	if (XR_FAILED(result)) {
		print_line("OpenXR: Failed to obtain Vulkan physical device [", OpenXRAPI::get_singleton()->get_error_string(result), "]");
		return false;
	}

	*r_device = vulkan_physical_device;

	return true;
}

bool OpenXRVulkanExtension::create_vulkan_device(const VkDeviceCreateInfo *p_device_create_info, VkDevice *r_device) {
	ERR_FAIL_NULL_V(OpenXRAPI::get_singleton(), false);

	XrVulkanDeviceCreateInfoKHR create_info = {
		XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR, // type
		nullptr, // next
		OpenXRAPI::get_singleton()->get_system_id(), // systemId
		0, // createFlags
		vkGetInstanceProcAddr, // pfnGetInstanceProcAddr
		vulkan_physical_device, // vulkanPhysicalDevice
		p_device_create_info, // vulkanCreateInfo
		nullptr // vulkanAllocator
	};

	VkResult vk_result = VK_SUCCESS;
	XrResult result = xrCreateVulkanDeviceKHR(OpenXRAPI::get_singleton()->get_instance(), &create_info, &vulkan_device, &vk_result);
	if (XR_FAILED(result)) {
		print_line("OpenXR: Failed to create Vulkan device [", OpenXRAPI::get_singleton()->get_error_string(result), "]");
		return false;
	}

	if (vk_result != VK_SUCCESS) {
		print_line("OpenXR: Failed to create Vulkan device [Vulkan error", vk_result, "]");
	}

	*r_device = vulkan_device;

	return true;
}

void OpenXRVulkanExtension::set_direct_queue_family_and_index(uint32_t p_queue_family_index, uint32_t p_queue_index) {
	vulkan_queue_family_index = p_queue_family_index;
	vulkan_queue_index = p_queue_index;
}

bool OpenXRVulkanExtension::use_fragment_density_offsets() {
	OpenXRFBFoveationExtension *fb_foveation = OpenXRFBFoveationExtension::get_singleton();
	if (fb_foveation == nullptr || is_deferred()) {
		return false;
	}

	return fb_foveation->is_foveation_eye_tracked_enabled();
}

void OpenXRVulkanExtension::get_fragment_density_offsets(LocalVector<VkOffset2D> &r_vk_offsets, const Vector2i &p_granularity) {
	OpenXRFBFoveationExtension *fb_foveation = OpenXRFBFoveationExtension::get_singleton();
	if (fb_foveation == nullptr) {
		return;
	}

	LocalVector<Vector2i> offsets;
	fb_foveation->get_fragment_density_offsets(offsets);

	r_vk_offsets.reserve(offsets.size());
	for (Vector2i offset : offsets) {
		offset = ((offset + p_granularity / 2) / p_granularity) * p_granularity;

		r_vk_offsets.push_back(VkOffset2D{ offset.x, offset.y });
	}
}

bool OpenXRVulkanExtension::use_subsampled_images() {
	OpenXRFBFoveationExtension *fb_foveation = OpenXRFBFoveationExtension::get_singleton();
	if (fb_foveation == nullptr || is_deferred()) {
		return false;
	}

	return fb_foveation->is_foveation_with_subsampled_images_enabled();
}

XrGraphicsBindingVulkanKHR OpenXRVulkanExtension::graphics_binding_vulkan;

void *OpenXRVulkanExtension::set_session_create_and_get_next_pointer(void *p_next_pointer) {
	DEV_ASSERT(vulkan_queue_family_index < UINT32_MAX && "Direct queue family index was not specified yet.");
	DEV_ASSERT(vulkan_queue_index < UINT32_MAX && "Direct queue index was not specified yet.");

	graphics_binding_vulkan.type = XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR;
	graphics_binding_vulkan.next = p_next_pointer;
	graphics_binding_vulkan.instance = vulkan_instance;
	graphics_binding_vulkan.physicalDevice = vulkan_physical_device;
	graphics_binding_vulkan.device = vulkan_device;
	graphics_binding_vulkan.queueFamilyIndex = vulkan_queue_family_index;
	graphics_binding_vulkan.queueIndex = vulkan_queue_index;

	return &graphics_binding_vulkan;
}

void OpenXRVulkanExtension::get_usable_swapchain_formats(Vector<int64_t> &p_usable_swap_chains) {
	// We might want to do more here especially if we keep things in linear color space
	// Possibly add in R10G10B10A2 as an option if we're using the mobile renderer.
	p_usable_swap_chains.push_back(VK_FORMAT_R8G8B8A8_SRGB);
	p_usable_swap_chains.push_back(VK_FORMAT_B8G8R8A8_SRGB);
	p_usable_swap_chains.push_back(VK_FORMAT_R8G8B8A8_UINT);
	p_usable_swap_chains.push_back(VK_FORMAT_B8G8R8A8_UINT);
}

void OpenXRVulkanExtension::get_usable_depth_formats(Vector<int64_t> &p_usable_swap_chains) {
	// Note, it is very likely we do NOT support any of depth formats where we can combine our stencil support (e.g. _S8_UINT).
	// Right now this isn't a problem but once stencil support becomes an issue, we need to check for this in the rendering engine
	// and create a separate buffer for the stencil.

	p_usable_swap_chains.push_back(VK_FORMAT_D24_UNORM_S8_UINT);
	p_usable_swap_chains.push_back(VK_FORMAT_D32_SFLOAT_S8_UINT);
	p_usable_swap_chains.push_back(VK_FORMAT_D32_SFLOAT);
}

bool OpenXRVulkanExtension::get_swapchain_image_data(XrSwapchain p_swapchain, int64_t p_swapchain_format, uint32_t p_width, uint32_t p_height, uint32_t p_sample_count, uint32_t p_array_size, void **r_swapchain_graphics_data) {
	LocalVector<XrSwapchainImageVulkanKHR> images;
	LocalVector<XrSwapchainImageFoveationVulkanFB> density_images;

	RenderingServer *rendering_server = RenderingServer::get_singleton();
	ERR_FAIL_NULL_V(rendering_server, false);
	RenderingDevice *rendering_device = rendering_server->get_rendering_device();
	ERR_FAIL_NULL_V(rendering_device, false);

	uint32_t swapchain_length;
	XrResult result = xrEnumerateSwapchainImages(p_swapchain, 0, &swapchain_length, nullptr);
	if (XR_FAILED(result)) {
		print_line("OpenXR: Failed to get swapchain image count [", OpenXRAPI::get_singleton()->get_error_string(result), "]");
		return false;
	}

	images.resize(swapchain_length);

	for (XrSwapchainImageVulkanKHR &image : images) {
		image.type = XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR;
		image.next = nullptr;
		image.image = VK_NULL_HANDLE;
	}

	// Deferred OpenXR renders on another device, the runtime's density maps don't apply.
	if (OpenXRFBFoveationExtension::get_singleton()->is_enabled() && !is_deferred()) {
		density_images.resize(swapchain_length);

		for (uint64_t i = 0; i < swapchain_length; i++) {
			density_images[i].type = XR_TYPE_SWAPCHAIN_IMAGE_FOVEATION_VULKAN_FB;
			density_images[i].next = nullptr;
			density_images[i].image = VK_NULL_HANDLE;
			density_images[i].width = 0;
			density_images[i].height = 0;

			images[i].next = &density_images[i];
		}
	}

	result = xrEnumerateSwapchainImages(p_swapchain, swapchain_length, &swapchain_length, (XrSwapchainImageBaseHeader *)images.ptr());
	if (XR_FAILED(result)) {
		print_line("OpenXR: Failed to get swapchain images [", OpenXRAPI::get_singleton()->get_error_string(result), "]");
		return false;
	}

	SwapchainGraphicsData *data = memnew(SwapchainGraphicsData);
	if (data == nullptr) {
		print_line("OpenXR: Failed to allocate memory for swapchain data");
		return false;
	}
	*r_swapchain_graphics_data = data;
	data->is_multiview = (p_array_size > 1);

	RenderingDevice::DataFormat format = RenderingDevice::DATA_FORMAT_R8G8B8A8_SRGB;
	RenderingDevice::TextureSamples samples = RenderingDevice::TEXTURE_SAMPLES_1;
	uint64_t usage_flags = RenderingDevice::TEXTURE_USAGE_SAMPLING_BIT;

	switch (p_swapchain_format) {
		case VK_FORMAT_R8G8B8A8_SRGB:
			// Even though this is an sRGB framebuffer format we're using UNORM here.
			// The reason here is because Godot does a linear to sRGB conversion while
			// with the sRGB format, this conversion would be doubled by the hardware.
			// This also means we're reading the values as is for our preview on screen.
			// The OpenXR runtime however is still treating this as an sRGB format and
			// will thus do an sRGB -> Linear conversion as expected.
			//format = RenderingDevice::DATA_FORMAT_R8G8B8A8_SRGB;
			format = RenderingDevice::DATA_FORMAT_R8G8B8A8_UNORM;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_COLOR_ATTACHMENT_BIT;
			break;
		case VK_FORMAT_B8G8R8A8_SRGB:
			//format = RenderingDevice::DATA_FORMAT_B8G8R8A8_SRGB;
			format = RenderingDevice::DATA_FORMAT_B8G8R8A8_UNORM;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_COLOR_ATTACHMENT_BIT;
			break;
		case VK_FORMAT_R8G8B8A8_UINT:
			format = RenderingDevice::DATA_FORMAT_R8G8B8A8_UINT;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_COLOR_ATTACHMENT_BIT;
			break;
		case VK_FORMAT_B8G8R8A8_UINT:
			format = RenderingDevice::DATA_FORMAT_B8G8R8A8_UINT;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_COLOR_ATTACHMENT_BIT;
			break;
		case VK_FORMAT_R16G16B16A16_SFLOAT:
			format = RenderingDevice::DATA_FORMAT_R16G16B16A16_SFLOAT;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_COLOR_ATTACHMENT_BIT;
			break;
		case VK_FORMAT_D32_SFLOAT:
			format = RenderingDevice::DATA_FORMAT_D32_SFLOAT;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | RenderingDevice::TEXTURE_USAGE_DEPTH_RESOLVE_ATTACHMENT_BIT;
			break;
		case VK_FORMAT_D24_UNORM_S8_UINT:
			format = RenderingDevice::DATA_FORMAT_D24_UNORM_S8_UINT;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | RenderingDevice::TEXTURE_USAGE_DEPTH_RESOLVE_ATTACHMENT_BIT;
			break;
		case VK_FORMAT_D32_SFLOAT_S8_UINT:
			format = RenderingDevice::DATA_FORMAT_D32_SFLOAT_S8_UINT;
			usage_flags |= RenderingDevice::TEXTURE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | RenderingDevice::TEXTURE_USAGE_DEPTH_RESOLVE_ATTACHMENT_BIT;
			break;
		default:
			// continue with our default value
			print_line("OpenXR: Unsupported swapchain format", p_swapchain_format);
			break;
	}

	switch (p_sample_count) {
		case 1:
			samples = RenderingDevice::TEXTURE_SAMPLES_1;
			break;
		case 2:
			samples = RenderingDevice::TEXTURE_SAMPLES_2;
			break;
		case 4:
			samples = RenderingDevice::TEXTURE_SAMPLES_4;
			break;
		case 8:
			samples = RenderingDevice::TEXTURE_SAMPLES_8;
			break;
		case 16:
			samples = RenderingDevice::TEXTURE_SAMPLES_16;
			break;
		case 32:
			samples = RenderingDevice::TEXTURE_SAMPLES_32;
			break;
		case 64:
			samples = RenderingDevice::TEXTURE_SAMPLES_64;
			break;
		default:
			// continue with our default value
			print_line("OpenXR: Unsupported sample count", p_sample_count);
			break;
	}

	if (is_deferred()) {
		VkFormat vk_format = VkFormat(p_swapchain_format);
		if (vk_format == VK_FORMAT_R8G8B8A8_SRGB) {
			vk_format = VK_FORMAT_R8G8B8A8_UNORM; // As above, Godot writes sRGB values itself.
		} else if (vk_format == VK_FORMAT_B8G8R8A8_SRGB) {
			vk_format = VK_FORMAT_B8G8R8A8_UNORM;
		}
		ERR_FAIL_COND_V_MSG(samples != RenderingDevice::TEXTURE_SAMPLES_1 || (usage_flags & RenderingDevice::TEXTURE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT), false, "OpenXR: deferred mode only copies single-sample color swapchains.");

		data->width = p_width;
		data->height = p_height;
		data->layers = p_array_size;
		data->deferred_images.resize(swapchain_length);
		for (uint32_t i = 0; i < swapchain_length; i++) {
			data->deferred_images[i].swapchain_image = images[i].image;
			if (!create_deferred_image(data->deferred_images[i], vk_format, format, p_width, p_height, p_array_size)) {
				return false;
			}
		}
		return true;
	}

	Vector<RID> texture_rids;
	Vector<RID> density_map_rids;

	// create Godot texture objects for each entry in our swapchain
	for (uint32_t i = 0; i < swapchain_length; i++) {
		const XrSwapchainImageVulkanKHR &swapchain_image = images[i];

		RID image_rid = rendering_device->texture_create_from_extension(
				p_array_size == 1 ? RenderingDevice::TEXTURE_TYPE_2D : RenderingDevice::TEXTURE_TYPE_2D_ARRAY,
				format,
				samples,
				usage_flags,
				(uint64_t)swapchain_image.image,
				p_width,
				p_height,
				1,
				p_array_size,
				1);

		texture_rids.push_back(image_rid);

		if (OpenXRFBFoveationExtension::get_singleton()->is_enabled() && density_images[i].image != VK_NULL_HANDLE) {
			RID density_map_rid = rendering_device->texture_create_from_extension(
					p_array_size == 1 ? RenderingDevice::TEXTURE_TYPE_2D : RenderingDevice::TEXTURE_TYPE_2D_ARRAY,
					RD::DATA_FORMAT_R8G8_UNORM,
					RenderingDevice::TEXTURE_SAMPLES_1,
					RD::TEXTURE_USAGE_COLOR_ATTACHMENT_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_VRS_ATTACHMENT_BIT,
					(uint64_t)density_images[i].image,
					density_images[i].width,
					density_images[i].height,
					1,
					p_array_size,
					1);

			density_map_rids.push_back(density_map_rid);
		} else {
			density_map_rids.push_back(RID());
		}
	}

	data->texture_rids = texture_rids;
	data->density_map_rids = density_map_rids;

	return true;
}

bool OpenXRVulkanExtension::create_projection_fov(const XrFovf p_fov, double p_z_near, double p_z_far, Projection &r_camera_matrix) {
	OpenXRUtil::XrMatrix4x4f matrix;
	OpenXRUtil::XrMatrix4x4f_CreateProjectionFov(&matrix, p_fov, (float)p_z_near, (float)p_z_far);

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 4; i++) {
			r_camera_matrix.columns[j][i] = matrix.m[j * 4 + i];
		}
	}

	return true;
}

RID OpenXRVulkanExtension::get_texture(void *p_swapchain_graphics_data, int p_image_index) {
	SwapchainGraphicsData *data = (SwapchainGraphicsData *)p_swapchain_graphics_data;
	ERR_FAIL_NULL_V(data, RID());

	if (!data->deferred_images.is_empty()) {
		ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_image_index, data->deferred_images.size(), RID());
		return data->deferred_images[p_image_index].render_rid;
	}

	ERR_FAIL_INDEX_V(p_image_index, data->texture_rids.size(), RID());
	return data->texture_rids[p_image_index];
}

RID OpenXRVulkanExtension::get_density_map(void *p_swapchain_graphics_data, int p_image_index) {
	SwapchainGraphicsData *data = (SwapchainGraphicsData *)p_swapchain_graphics_data;
	ERR_FAIL_NULL_V(data, RID());

	ERR_FAIL_INDEX_V(p_image_index, data->density_map_rids.size(), RID());
	return data->density_map_rids[p_image_index];
}

void OpenXRVulkanExtension::cleanup_swapchain_graphics_data(void **p_swapchain_graphics_data) {
	if (*p_swapchain_graphics_data == nullptr) {
		return;
	}

	SwapchainGraphicsData *data = (SwapchainGraphicsData *)*p_swapchain_graphics_data;

	RenderingServer *rendering_server = RenderingServer::get_singleton();
	ERR_FAIL_NULL(rendering_server);
	RenderingDevice *rendering_device = rendering_server->get_rendering_device();
	ERR_FAIL_NULL(rendering_device);

	for (const RID &texture_rid : data->texture_rids) {
		// This should clean up our RIDs and associated texture objects but shouldn't destroy the images, they are owned by our XrSwapchain.
		rendering_device->free_rid(texture_rid);
	}
	data->texture_rids.clear();

	for (int i = 0; i < data->density_map_rids.size(); i++) {
		if (data->density_map_rids[i].is_valid()) {
			rendering_device->free_rid(data->density_map_rids[i]);
		}
	}
	data->density_map_rids.clear();

	if (!data->deferred_images.is_empty()) {
		// Both devices may still be using the images.
		vkQueueWaitIdle(godot_queue);
		vkDeviceWaitIdle(vulkan_device);
		for (DeferredImage &image : data->deferred_images) {
			free_deferred_image(image);
		}
		data->deferred_images.clear();
	}

	memdelete(data);
	*p_swapchain_graphics_data = nullptr;
}

bool OpenXRVulkanExtension::is_deferred() const {
	return OpenXRAPI::get_singleton() != nullptr && OpenXRAPI::get_singleton()->is_deferred();
}

static uint32_t _find_memory_type(VkPhysicalDevice p_physical_device, uint32_t p_type_bits) {
	VkPhysicalDeviceMemoryProperties properties;
	vkGetPhysicalDeviceMemoryProperties(p_physical_device, &properties);
	for (uint32_t i = 0; i < properties.memoryTypeCount; i++) {
		if ((p_type_bits & (1u << i)) && (properties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)) {
			return i;
		}
	}
	return UINT32_MAX;
}

bool OpenXRVulkanExtension::create_deferred_graphics() {
#ifdef WINDOWS_ENABLED
	RenderingDevice *rendering_device = RenderingServer::get_singleton()->get_rendering_device();
	ERR_FAIL_NULL_V(rendering_device, false);
	godot_device = (VkDevice)rendering_device->get_driver_resource(RenderingDevice::DRIVER_RESOURCE_LOGICAL_DEVICE);
	godot_physical_device = (VkPhysicalDevice)rendering_device->get_driver_resource(RenderingDevice::DRIVER_RESOURCE_PHYSICAL_DEVICE);
	godot_queue = (VkQueue)rendering_device->get_driver_resource(RenderingDevice::DRIVER_RESOURCE_COMMAND_QUEUE);

	get_memory_win32_handle = vkGetDeviceProcAddr(godot_device, "vkGetMemoryWin32HandleKHR");
	if (get_memory_win32_handle == nullptr) {
		print_line("OpenXR: the renderer's Vulkan device lacks VK_KHR_external_memory_win32, can't share images with the runtime");
		return false;
	}

	// Our own instance and device, created by the runtime as XR_KHR_vulkan_enable2 requires.
	VkApplicationInfo app_info = {};
	app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	app_info.pApplicationName = "Godot OpenXR";
	app_info.apiVersion = VK_API_VERSION_1_2;
	VkInstanceCreateInfo instance_info = {};
	instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	instance_info.pApplicationInfo = &app_info;
	if (!create_vulkan_instance(&instance_info, &vulkan_instance)) {
		return false;
	}
	if (!get_physical_device(&vulkan_physical_device)) {
		return false;
	}

	// Images can only be shared on one GPU.
	VkPhysicalDeviceIDProperties ids[2] = {};
	const VkPhysicalDevice physical_devices[2] = { godot_physical_device, vulkan_physical_device };
	for (int i = 0; i < 2; i++) {
		ids[i].sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES;
		VkPhysicalDeviceProperties2 properties = {};
		properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
		properties.pNext = &ids[i];
		vkGetPhysicalDeviceProperties2(physical_devices[i], &properties);
	}
	if (memcmp(ids[0].deviceUUID, ids[1].deviceUUID, VK_UUID_SIZE) != 0) {
		print_line("OpenXR: the headset runs on a different GPU than the renderer, can't share images");
		return false;
	}

	uint32_t family_count = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(vulkan_physical_device, &family_count, nullptr);
	LocalVector<VkQueueFamilyProperties> families;
	families.resize(family_count);
	vkGetPhysicalDeviceQueueFamilyProperties(vulkan_physical_device, &family_count, families.ptr());
	vulkan_queue_family_index = UINT32_MAX;
	for (uint32_t i = 0; i < family_count; i++) {
		if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
			vulkan_queue_family_index = i;
			break;
		}
	}
	ERR_FAIL_COND_V(vulkan_queue_family_index == UINT32_MAX, false);
	vulkan_queue_index = 0;

	const float priority = 1.0f;
	VkDeviceQueueCreateInfo queue_info = {};
	queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queue_info.queueFamilyIndex = vulkan_queue_family_index;
	queue_info.queueCount = 1;
	queue_info.pQueuePriorities = &priority;
	const char *device_extensions[] = { VK_KHR_EXTERNAL_MEMORY_WIN32_EXTENSION_NAME };
	VkDeviceCreateInfo device_info = {};
	device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	device_info.queueCreateInfoCount = 1;
	device_info.pQueueCreateInfos = &queue_info;
	device_info.enabledExtensionCount = 1;
	device_info.ppEnabledExtensionNames = device_extensions;
	if (!create_vulkan_device(&device_info, &vulkan_device) || vulkan_device == nullptr) {
		return false;
	}
	vkGetDeviceQueue(vulkan_device, vulkan_queue_family_index, vulkan_queue_index, &deferred_queue);

	VkCommandPoolCreateInfo pool_info = {};
	pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	pool_info.queueFamilyIndex = vulkan_queue_family_index;
	ERR_FAIL_COND_V(vkCreateCommandPool(vulkan_device, &pool_info, nullptr, &deferred_command_pool) != VK_SUCCESS, false);
	VkCommandBufferAllocateInfo buffer_info = {};
	buffer_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	buffer_info.commandPool = deferred_command_pool;
	buffer_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	buffer_info.commandBufferCount = 1;
	ERR_FAIL_COND_V(vkAllocateCommandBuffers(vulkan_device, &buffer_info, &deferred_command_buffer) != VK_SUCCESS, false);
	VkFenceCreateInfo fence_info = {};
	fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	ERR_FAIL_COND_V(vkCreateFence(vulkan_device, &fence_info, nullptr, &deferred_fence) != VK_SUCCESS, false);

	print_line("OpenXR: deferred, frames are copied to the runtime's Vulkan device");
	return true;
#else
	print_line("OpenXR: deferred mode is only implemented on Windows");
	return false;
#endif
}

void OpenXRVulkanExtension::destroy_deferred_graphics() {
	if (vulkan_device != nullptr) {
		vkDeviceWaitIdle(vulkan_device);
		if (deferred_fence != VK_NULL_HANDLE) {
			vkDestroyFence(vulkan_device, deferred_fence, nullptr);
		}
		if (deferred_command_pool != VK_NULL_HANDLE) {
			vkDestroyCommandPool(vulkan_device, deferred_command_pool, nullptr);
		}
		vkDestroyDevice(vulkan_device, nullptr);
	}
	if (vulkan_instance != nullptr) {
		vkDestroyInstance(vulkan_instance, nullptr);
	}
	deferred_fence = VK_NULL_HANDLE;
	deferred_command_pool = VK_NULL_HANDLE;
	deferred_command_buffer = nullptr;
	deferred_queue = nullptr;
	vulkan_device = nullptr;
	vulkan_physical_device = nullptr;
	vulkan_instance = nullptr;
}

bool OpenXRVulkanExtension::create_deferred_image(DeferredImage &r_image, VkFormat p_vk_format, int p_rd_format, uint32_t p_width, uint32_t p_height, uint32_t p_layers) {
#ifdef WINDOWS_ENABLED
	RenderingDevice *rendering_device = RenderingServer::get_singleton()->get_rendering_device();
	ERR_FAIL_NULL_V(rendering_device, false);

	// Both devices create the image the same way; the memory is allocated on the renderer's and imported on ours.
	VkExternalMemoryImageCreateInfo external_info = {};
	external_info.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO;
	external_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
	VkImageCreateInfo image_info = {};
	image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	image_info.pNext = &external_info;
	image_info.imageType = VK_IMAGE_TYPE_2D;
	image_info.format = p_vk_format;
	image_info.extent = { p_width, p_height, 1 };
	image_info.mipLevels = 1;
	image_info.arrayLayers = p_layers;
	image_info.samples = VK_SAMPLE_COUNT_1_BIT;
	image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
	image_info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
	image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	ERR_FAIL_COND_V(vkCreateImage(godot_device, &image_info, nullptr, &r_image.shared_image) != VK_SUCCESS, false);
	VkMemoryRequirements requirements;
	vkGetImageMemoryRequirements(godot_device, r_image.shared_image, &requirements);
	uint32_t memory_type = _find_memory_type(godot_physical_device, requirements.memoryTypeBits);
	ERR_FAIL_COND_V(memory_type == UINT32_MAX, false);

	VkMemoryDedicatedAllocateInfo dedicated_info = {};
	dedicated_info.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
	dedicated_info.image = r_image.shared_image;
	VkExportMemoryAllocateInfo export_info = {};
	export_info.sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO;
	export_info.pNext = &dedicated_info;
	export_info.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
	VkMemoryAllocateInfo allocate_info = {};
	allocate_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocate_info.pNext = &export_info;
	allocate_info.allocationSize = requirements.size;
	allocate_info.memoryTypeIndex = memory_type;
	ERR_FAIL_COND_V(vkAllocateMemory(godot_device, &allocate_info, nullptr, &r_image.shared_memory) != VK_SUCCESS, false);
	ERR_FAIL_COND_V(vkBindImageMemory(godot_device, r_image.shared_image, r_image.shared_memory, 0) != VK_SUCCESS, false);

	VkMemoryGetWin32HandleInfoKHR handle_info = {};
	handle_info.sType = VK_STRUCTURE_TYPE_MEMORY_GET_WIN32_HANDLE_INFO_KHR;
	handle_info.memory = r_image.shared_memory;
	handle_info.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
	HANDLE handle = nullptr;
	ERR_FAIL_COND_V(((PFN_vkGetMemoryWin32HandleKHR)get_memory_win32_handle)(godot_device, &handle_info, &handle) != VK_SUCCESS, false);

	VkResult result = vkCreateImage(vulkan_device, &image_info, nullptr, &r_image.imported_image);
	if (result == VK_SUCCESS) {
		VkMemoryDedicatedAllocateInfo import_dedicated_info = {};
		import_dedicated_info.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
		import_dedicated_info.image = r_image.imported_image;
		VkImportMemoryWin32HandleInfoKHR import_info = {};
		import_info.sType = VK_STRUCTURE_TYPE_IMPORT_MEMORY_WIN32_HANDLE_INFO_KHR;
		import_info.pNext = &import_dedicated_info;
		import_info.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
		import_info.handle = handle;
		allocate_info.pNext = &import_info; // Same size and memory type: same GPU, same driver.
		result = vkAllocateMemory(vulkan_device, &allocate_info, nullptr, &r_image.imported_memory);
		if (result == VK_SUCCESS) {
			result = vkBindImageMemory(vulkan_device, r_image.imported_image, r_image.imported_memory, 0);
		}
	}
	CloseHandle(handle); // Importing an opaque Win32 handle doesn't take ownership of it.
	ERR_FAIL_COND_V_MSG(result != VK_SUCCESS, false, vformat("OpenXR: couldn't import the shared image on the runtime's device (VkResult %d).", result));

	RenderingDevice::DataFormat format = RenderingDevice::DataFormat(p_rd_format);
	RenderingDevice::TextureType type = p_layers == 1 ? RenderingDevice::TEXTURE_TYPE_2D : RenderingDevice::TEXTURE_TYPE_2D_ARRAY;
	r_image.shared_rid = rendering_device->texture_create_from_extension(type, format, RenderingDevice::TEXTURE_SAMPLES_1, RenderingDevice::TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice::TEXTURE_USAGE_CAN_COPY_TO_BIT, (uint64_t)r_image.shared_image, p_width, p_height, 1, p_layers, 1);
	ERR_FAIL_COND_V(r_image.shared_rid.is_null(), false);

	RenderingDevice::TextureFormat texture_format;
	texture_format.format = format;
	texture_format.width = p_width;
	texture_format.height = p_height;
	texture_format.array_layers = p_layers;
	texture_format.texture_type = type;
	texture_format.usage_bits = RenderingDevice::TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice::TEXTURE_USAGE_COLOR_ATTACHMENT_BIT | RenderingDevice::TEXTURE_USAGE_CAN_COPY_FROM_BIT;
	if (format == RenderingDevice::DATA_FORMAT_R8G8B8A8_UNORM) {
		// Swapchain textures allow sRGB views of the same data, keep that.
		texture_format.shareable_formats.push_back(RenderingDevice::DATA_FORMAT_R8G8B8A8_UNORM);
		texture_format.shareable_formats.push_back(RenderingDevice::DATA_FORMAT_R8G8B8A8_SRGB);
	}
	r_image.render_rid = rendering_device->texture_create(texture_format, RenderingDevice::TextureView());
	ERR_FAIL_COND_V(r_image.render_rid.is_null(), false);
	return true;
#else
	return false;
#endif
}

void OpenXRVulkanExtension::free_deferred_image(DeferredImage &r_image) {
	RenderingDevice *rendering_device = RenderingServer::get_singleton()->get_rendering_device();
	if (rendering_device != nullptr) {
		if (r_image.render_rid.is_valid()) {
			rendering_device->free_rid(r_image.render_rid);
		}
		if (r_image.shared_rid.is_valid()) {
			rendering_device->free_rid(r_image.shared_rid);
		}
	}
	if (r_image.imported_image != VK_NULL_HANDLE) {
		vkDestroyImage(vulkan_device, r_image.imported_image, nullptr);
	}
	if (r_image.imported_memory != VK_NULL_HANDLE) {
		vkFreeMemory(vulkan_device, r_image.imported_memory, nullptr);
	}
	if (r_image.shared_image != VK_NULL_HANDLE) {
		vkDestroyImage(godot_device, r_image.shared_image, nullptr);
	}
	if (r_image.shared_memory != VK_NULL_HANDLE) {
		vkFreeMemory(godot_device, r_image.shared_memory, nullptr);
	}
	r_image = DeferredImage();
}

void OpenXRVulkanExtension::on_post_draw_swapchain(void *p_swapchain_graphics_data, int p_image_index) {
	SwapchainGraphicsData *data = (SwapchainGraphicsData *)p_swapchain_graphics_data;
	if (data == nullptr || data->deferred_images.is_empty()) {
		return;
	}
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_image_index, data->deferred_images.size());
	const DeferredImage &image = data->deferred_images[p_image_index];

	// Ends the frame with the shared image in TRANSFER_DST_OPTIMAL, which is where our copy picks it up.
	RenderingDevice *rendering_device = RenderingServer::get_singleton()->get_rendering_device();
	for (uint32_t layer = 0; layer < data->layers; layer++) {
		rendering_device->texture_copy(image.render_rid, image.shared_rid, Vector3(), Vector3(), Vector3(data->width, data->height, 1), 0, 0, layer, layer);
	}
}

void OpenXRVulkanExtension::on_swapchain_release(void *p_swapchain_graphics_data, int p_image_index) {
	SwapchainGraphicsData *data = (SwapchainGraphicsData *)p_swapchain_graphics_data;
	if (data == nullptr || data->deferred_images.is_empty()) {
		return;
	}
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_image_index, data->deferred_images.size());
	const DeferredImage &image = data->deferred_images[p_image_index];

	// The renderer has submitted this frame (XRServer::end_frame); wait for it on the CPU.
	// TODO: a shared semaphore instead, so the renderer and our copy can overlap.
	const uint64_t wait_start = OS::get_singleton()->get_ticks_usec();
	vkQueueWaitIdle(godot_queue);
	const uint64_t copy_start = OS::get_singleton()->get_ticks_usec();

	VkCommandBufferBeginInfo begin_info = {};
	begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
	vkBeginCommandBuffer(deferred_command_buffer, &begin_info);

	VkImageSubresourceRange range = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, data->layers };
	VkImageMemoryBarrier before[2] = {};
	before[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	before[0].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	before[0].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	before[0].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	before[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_EXTERNAL;
	before[0].dstQueueFamilyIndex = vulkan_queue_family_index;
	before[0].image = image.imported_image;
	before[0].subresourceRange = range;
	before[1].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	before[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	before[1].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
	before[1].oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; // How OpenXR hands out color swapchain images.
	before[1].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	before[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	before[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	before[1].image = image.swapchain_image;
	before[1].subresourceRange = range;
	vkCmdPipelineBarrier(deferred_command_buffer, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 2, before);

	VkImageCopy region = {};
	region.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, data->layers };
	region.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, data->layers };
	region.extent = { data->width, data->height, 1 };
	vkCmdCopyImage(deferred_command_buffer, image.imported_image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image.swapchain_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

	VkImageMemoryBarrier after[2] = {};
	after[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	after[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
	after[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	after[0].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	after[0].newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL; // How OpenXR wants it back.
	after[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	after[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	after[0].image = image.swapchain_image;
	after[0].subresourceRange = range;
	after[1].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	after[1].srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	after[1].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
	after[1].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; // Where the renderer's next copy expects it.
	after[1].srcQueueFamilyIndex = vulkan_queue_family_index;
	after[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_EXTERNAL;
	after[1].image = image.imported_image;
	after[1].subresourceRange = range;
	vkCmdPipelineBarrier(deferred_command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, nullptr, 0, nullptr, 2, after);

	vkEndCommandBuffer(deferred_command_buffer);

	VkSubmitInfo submit_info = {};
	submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit_info.commandBufferCount = 1;
	submit_info.pCommandBuffers = &deferred_command_buffer;
	vkQueueSubmit(deferred_queue, 1, &submit_info, deferred_fence);
	vkWaitForFences(vulkan_device, 1, &deferred_fence, VK_TRUE, UINT64_MAX);
	vkResetFences(vulkan_device, 1, &deferred_fence);

	// `--verbose`: what the CPU wait and the copy cost, averaged over about 5 s.
	if (OS::get_singleton()->is_stdout_verbose()) {
		const uint64_t end = OS::get_singleton()->get_ticks_usec();
		wait_usec += copy_start - wait_start;
		copy_usec += end - copy_start;
		if (++timed_frames == 450) {
			print_line(vformat("OpenXR: deferred, per frame: waiting for the renderer %.2f ms, copying to the runtime %.2f ms", wait_usec / 450000.0, copy_usec / 450000.0));
			wait_usec = 0;
			copy_usec = 0;
			timed_frames = 0;
		}
	}
}

#define ENUM_TO_STRING_CASE(e) \
	case e: { \
		return String(#e); \
	} break;

String OpenXRVulkanExtension::get_swapchain_format_name(int64_t p_swapchain_format) const {
	// This really should be in vulkan_context...
	VkFormat format = VkFormat(p_swapchain_format);
	switch (format) {
		ENUM_TO_STRING_CASE(VK_FORMAT_UNDEFINED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R4G4_UNORM_PACK8)
		ENUM_TO_STRING_CASE(VK_FORMAT_R4G4B4A4_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_B4G4R4A4_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R5G6B5_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_B5G6R5_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R5G5B5A1_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_B5G5R5A1_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_A1R5G5B5_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8_SRGB)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8_SRGB)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8_SRGB)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8_SRGB)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8A8_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8A8_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8A8_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8A8_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8A8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8A8_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R8G8B8A8_SRGB)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8A8_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8A8_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8A8_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8A8_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8A8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8A8_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8A8_SRGB)
		ENUM_TO_STRING_CASE(VK_FORMAT_A8B8G8R8_UNORM_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A8B8G8R8_SNORM_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A8B8G8R8_USCALED_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A8B8G8R8_SSCALED_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A8B8G8R8_UINT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A8B8G8R8_SINT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A8B8G8R8_SRGB_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2R10G10B10_UNORM_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2R10G10B10_SNORM_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2R10G10B10_USCALED_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2R10G10B10_SSCALED_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2R10G10B10_UINT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2R10G10B10_SINT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2B10G10R10_UNORM_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2B10G10R10_SNORM_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2B10G10R10_USCALED_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2B10G10R10_SSCALED_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2B10G10R10_UINT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_A2B10G10R10_SINT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16A16_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16A16_SNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16A16_USCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16A16_SSCALED)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16A16_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16A16_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R16G16B16A16_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32B32_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32B32_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32B32_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32B32A32_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32B32A32_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R32G32B32A32_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64B64_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64B64_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64B64_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64B64A64_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64B64A64_SINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_R64G64B64A64_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_B10G11R11_UFLOAT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_E5B9G9R9_UFLOAT_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_D16_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_X8_D24_UNORM_PACK32)
		ENUM_TO_STRING_CASE(VK_FORMAT_D32_SFLOAT)
		ENUM_TO_STRING_CASE(VK_FORMAT_S8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_D16_UNORM_S8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_D24_UNORM_S8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_D32_SFLOAT_S8_UINT)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC1_RGB_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC1_RGB_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC1_RGBA_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC1_RGBA_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC2_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC2_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC3_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC3_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC4_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC4_SNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC5_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC5_SNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC6H_UFLOAT_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC6H_SFLOAT_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC7_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_BC7_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ETC2_R8G8B8_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ETC2_R8G8B8_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ETC2_R8G8B8A1_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ETC2_R8G8B8A1_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ETC2_R8G8B8A8_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ETC2_R8G8B8A8_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_EAC_R11_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_EAC_R11_SNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_EAC_R11G11_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_EAC_R11G11_SNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_4x4_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_4x4_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_5x4_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_5x4_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_5x5_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_5x5_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_6x5_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_6x5_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_6x6_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_6x6_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x5_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x5_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x6_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x6_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x8_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x8_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x5_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x5_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x6_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x6_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x8_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x8_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x10_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x10_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_12x10_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_12x10_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_12x12_UNORM_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_12x12_SRGB_BLOCK)
		ENUM_TO_STRING_CASE(VK_FORMAT_G8B8G8R8_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_B8G8R8G8_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G8_B8_R8_3PLANE_420_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G8_B8R8_2PLANE_420_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G8_B8_R8_3PLANE_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G8_B8R8_2PLANE_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G8_B8_R8_3PLANE_444_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_R10X6_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R10X6G10X6_UNORM_2PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R10X6G10X6B10X6A10X6_UNORM_4PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G10X6B10X6G10X6R10X6_422_UNORM_4PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_B10X6G10X6R10X6G10X6_422_UNORM_4PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G10X6_B10X6_R10X6_3PLANE_420_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G10X6_B10X6R10X6_2PLANE_420_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G10X6_B10X6_R10X6_3PLANE_422_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G10X6_B10X6R10X6_2PLANE_422_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G10X6_B10X6_R10X6_3PLANE_444_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R12X4_UNORM_PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R12X4G12X4_UNORM_2PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_R12X4G12X4B12X4A12X4_UNORM_4PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G12X4B12X4G12X4R12X4_422_UNORM_4PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_B12X4G12X4R12X4G12X4_422_UNORM_4PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G12X4_B12X4_R12X4_3PLANE_420_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G12X4_B12X4R12X4_2PLANE_420_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G12X4_B12X4_R12X4_3PLANE_422_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G12X4_B12X4R12X4_2PLANE_422_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G12X4_B12X4_R12X4_3PLANE_444_UNORM_3PACK16)
		ENUM_TO_STRING_CASE(VK_FORMAT_G16B16G16R16_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_B16G16R16G16_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G16_B16_R16_3PLANE_420_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G16_B16R16_2PLANE_420_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G16_B16_R16_3PLANE_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G16_B16R16_2PLANE_422_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_G16_B16_R16_3PLANE_444_UNORM)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC1_2BPP_UNORM_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC1_4BPP_UNORM_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC2_2BPP_UNORM_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC2_4BPP_UNORM_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC1_2BPP_SRGB_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC1_4BPP_SRGB_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC2_2BPP_SRGB_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_PVRTC2_4BPP_SRGB_BLOCK_IMG)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_4x4_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_5x4_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_5x5_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_6x5_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_6x6_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x5_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x6_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_8x8_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x5_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x6_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x8_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_10x10_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_12x10_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_ASTC_12x12_SFLOAT_BLOCK_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_G8_B8R8_2PLANE_444_UNORM_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_G10X6_B10X6R10X6_2PLANE_444_UNORM_3PACK16_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_G12X4_B12X4R12X4_2PLANE_444_UNORM_3PACK16_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_G16_B16R16_2PLANE_444_UNORM_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_A4R4G4B4_UNORM_PACK16_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_A4B4G4R4_UNORM_PACK16_EXT)
		ENUM_TO_STRING_CASE(VK_FORMAT_MAX_ENUM)
		default: {
			return String("Swapchain format ") + String::num_int64(int64_t(p_swapchain_format));
		} break;
	}
}
