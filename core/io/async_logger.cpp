/**************************************************************************/
/*  async_logger.cpp                                                      */
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

#include "async_logger.h"

#include <cstdio>

AsyncLogger::AsyncLogger(Logger *p_inner) :
		inner(p_inner) {
}

// Caller holds write_mutex.
void AsyncLogger::_write_queued() {
	LocalVector<CharString> lines;
	{
		MutexLock lock(queue_mutex);
		SWAP(lines, queue);
	}
	for (const CharString &line : lines) {
		inner->logf("%s", line.get_data());
	}
}

void AsyncLogger::_thread_func(void *p_self) {
	AsyncLogger *self = static_cast<AsyncLogger *>(p_self);
	while (true) {
		self->pending.wait();
		MutexLock lock(self->write_mutex);
		self->_write_queued();
		if (self->exiting.is_set()) {
			return;
		}
	}
}

void AsyncLogger::logv(const char *p_format, va_list p_list, bool p_err) {
	if (!should_log(p_err)) {
		return;
	}

	if (p_err) {
		MutexLock lock(write_mutex);
		_write_queued();
		inner->logv(p_format, p_list, true);
		return;
	}

	va_list list_copy;
	va_copy(list_copy, p_list);
	int len = vsnprintf(nullptr, 0, p_format, list_copy);
	va_end(list_copy);
	if (len <= 0) {
		return;
	}
	CharString line;
	line.resize_uninitialized(len + 1);
	vsnprintf(line.ptrw(), len + 1, p_format, p_list);

	{
		MutexLock lock(queue_mutex);
		queue.push_back(line);
		// Started on first use, not in the constructor: loggers exist before threads can be named.
		if (!started.is_set()) {
			started.set();
			thread.start(_thread_func, this);
		}
	}
	pending.post();
}

void AsyncLogger::log_error(const char *p_function, const char *p_file, int p_line, const char *p_code, const char *p_rationale, bool p_editor_notify, ErrorType p_type, const Vector<Ref<ScriptBacktrace>> &p_script_backtraces) {
	if (!should_log(true)) {
		return;
	}

	MutexLock lock(write_mutex);
	_write_queued();
	inner->log_error(p_function, p_file, p_line, p_code, p_rationale, p_editor_notify, p_type, p_script_backtraces);
}

AsyncLogger::~AsyncLogger() {
	if (started.is_set()) {
		exiting.set();
		pending.post();
		thread.wait_to_finish();
	}
	MutexLock lock(write_mutex);
	_write_queued();
	memdelete(inner);
}
