/**************************************************************************/
/*  async_logger.h                                                        */
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

#pragma once

#include "core/io/logger.h"
#include "core/os/mutex.h"
#include "core/os/semaphore.h"
#include "core/os/thread.h"
#include "core/templates/local_vector.h"
#include "core/templates/safe_refcount.h"

/**
 * Writes non-error lines to the wrapped logger on a thread of its own, so a print never waits
 * on I/O. Errors wait: they first write out everything queued before them, then themselves.
 */
class AsyncLogger : public Logger {
	Logger *inner = nullptr;
	Thread thread;
	Mutex queue_mutex; // Held only to push to or take the queue.
	Mutex write_mutex; // Held for every write to `inner`, so lines keep their order.
	Semaphore pending;
	LocalVector<CharString> queue;
	SafeFlag started;
	SafeFlag exiting;

	void _write_queued();
	static void _thread_func(void *p_self);

public:
	explicit AsyncLogger(Logger *p_inner);

	virtual void logv(const char *p_format, va_list p_list, bool p_err) override _PRINTF_FORMAT_ATTRIBUTE_2_0;
	virtual void log_error(const char *p_function, const char *p_file, int p_line, const char *p_code, const char *p_rationale, bool p_editor_notify, ErrorType p_type = ERR_ERROR, const Vector<Ref<ScriptBacktrace>> &p_script_backtraces = {}) override;

	virtual ~AsyncLogger();
};
