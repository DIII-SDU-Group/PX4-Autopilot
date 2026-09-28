/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "externalChecks.hpp"

#include <cstring>

namespace {
uint32_t next_external_checks_instance_generation{0};
}

ExternalChecks::ExternalChecks()
	: _diagnostic_px4_instance_generation(++next_external_checks_instance_generation)
{
}

const char *ExternalChecks::diagnosticDispositionName(DiagnosticDisposition disposition)
{
	switch (disposition) {
	case DiagnosticDisposition::RequestPublished:
		return "REQUEST_PUBLISHED";
	case DiagnosticDisposition::RegistrationAllocated:
		return "REGISTRATION_ALLOCATED";
	case DiagnosticDisposition::RegistrationFreed:
		return "REGISTRATION_FREED";
	case DiagnosticDisposition::ReplyObserved:
		return "REPLY_OBSERVED";
	case DiagnosticDisposition::Accepted:
		return "ACCEPTED";
	case DiagnosticDisposition::RejectedStaleRequestId:
		return "REJECTED_STALE_REQUEST_ID";
	case DiagnosticDisposition::RejectedUnknownRegistration:
		return "REJECTED_UNKNOWN_REGISTRATION";
	case DiagnosticDisposition::RejectedOther:
		return "REJECTED_OTHER";
	case DiagnosticDisposition::RegistrationUnresponsive:
		return "REGISTRATION_UNRESPONSIVE";
	case DiagnosticDisposition::FailureReported:
		return "FAILURE_REPORTED";
	}

	return "UNKNOWN";
}

void ExternalChecks::recordDiagnosticEvent(const DiagnosticEvent &event)
{
	DiagnosticEvent stored = event;
	stored.sequence = _diagnostic_next_sequence++;
	_diagnostic_history[_diagnostic_history_next] = stored;
	_diagnostic_history_next = (_diagnostic_history_next + 1) % DIAGNOSTIC_HISTORY_SIZE;
	if (_diagnostic_history_count < DIAGNOSTIC_HISTORY_SIZE) {
		++_diagnostic_history_count;
	}
}

void ExternalChecks::dumpDiagnosticHistory()
{
	if (_diagnostic_failure_dumped) {
		return;
	}

	_diagnostic_failure_dumped = true;
	PX4_WARN("[HIL-WO002] external-check history begin count=%lu active=0x%08lx received=0x%08lx current_request=%u",
		 (unsigned long)_diagnostic_history_count,
		 (unsigned long)_active_registrations_mask,
		 (unsigned long)_reply_received_mask,
		 (unsigned)_current_request_id);

	const uint32_t first = _diagnostic_history_count == DIAGNOSTIC_HISTORY_SIZE ? _diagnostic_history_next : 0;

	for (uint32_t offset = 0; offset < _diagnostic_history_count; ++offset) {
		const uint32_t index = (first + offset) % DIAGNOSTIC_HISTORY_SIZE;
		const DiagnosticEvent &event = _diagnostic_history[index];
		PX4_INFO(
			"[HIL-WO002] seq=%lu kind=%s t_us=%llu reg=%u reply_req=%u expected_req=%u "
			"px4_gen=%lu req_seq=%lu reg_gen=%lu nav=%d replaces=%d name=%s "
			"active=0x%08lx before=0x%08lx after=0x%08lx no_reply=0x%08lx "
			"gap_us=%llu noresp=%u can_arm=%u duplicate=%u first=%u health=%u events=%u reply_ts=%llu",
			(unsigned long)event.sequence,
			diagnosticDispositionName(event.disposition),
			(unsigned long long)event.timestamp,
			(unsigned)event.registration_id,
			(unsigned)event.reply_request_id,
			(unsigned)event.expected_request_id,
			(unsigned long)event.px4_instance_generation,
			(unsigned long)event.request_publish_sequence,
			(unsigned long)event.registration_generation,
			(int)event.registration_nav_mode_id,
			(int)event.registration_replaces_nav_state,
			event.registration_name,
			(unsigned long)event.active_registration_mask,
			(unsigned long)event.received_mask_before,
			(unsigned long)event.received_mask_after,
			(unsigned long)event.no_reply_mask,
			(unsigned long long)event.time_since_last_accepted_reply_us,
			(unsigned)event.num_no_response,
			event.can_arm_and_run ? 1u : 0u,
			event.duplicate_current_reply ? 1u : 0u,
			event.waiting_for_first_response ? 1u : 0u,
			(unsigned)event.health_component_index,
			(unsigned)event.num_events,
			(unsigned long long)event.reply_timestamp);
	}

	PX4_WARN("[HIL-WO002] external-check history end");
}

static void setOrClearRequirementBits(bool requirement_set, int8_t nav_state, int8_t replaces_nav_state, uint32_t &bits)
{
	if (requirement_set) {
		bits |= 1u << nav_state;
	}

	if (replaces_nav_state != -1) {
		if (requirement_set) {
			bits |= 1u << replaces_nav_state;

		} else {
			bits &= ~(1u << replaces_nav_state);
		}
	}
}

int ExternalChecks::addRegistration(int8_t nav_mode_id, int8_t replaces_nav_state,
					const char *registration_name)
{
	int free_registration_index = -1;

	for (int i = 0; i < MAX_NUM_REGISTRATIONS; ++i) {
		if (!registrationValid(i)) {
			free_registration_index = i;
			break;
		}
	}

	if (free_registration_index != -1) {
		_active_registrations_mask |= 1 << free_registration_index;
		_registrations[free_registration_index].nav_mode_id = nav_mode_id;
		_registrations[free_registration_index].replaces_nav_state = replaces_nav_state;
		_registrations[free_registration_index].generation++;
		memset(_registrations[free_registration_index].name, 0, sizeof(_registrations[free_registration_index].name));

		if (registration_name) {
			strncpy(_registrations[free_registration_index].name, registration_name,
				sizeof(_registrations[free_registration_index].name) - 1);
		}
		_registrations[free_registration_index].waiting_for_first_response = true;
		_registrations[free_registration_index].num_no_response = 0;
		_registrations[free_registration_index].unresponsive = false;
		_registrations[free_registration_index].total_num_unresponsive = 0;
		_last_accepted_reply_time[free_registration_index] = 0;

		if (!_registrations[free_registration_index].reply) {
			_registrations[free_registration_index].reply = new arming_check_reply_s();
		}

		DiagnosticEvent allocated{};
		allocated.timestamp = hrt_absolute_time();
		allocated.disposition = DiagnosticDisposition::RegistrationAllocated;
		allocated.registration_id = static_cast<uint8_t>(free_registration_index);
		allocated.registration_nav_mode_id = nav_mode_id;
		allocated.registration_replaces_nav_state = replaces_nav_state;
		allocated.registration_generation = _registrations[free_registration_index].generation;
		allocated.px4_instance_generation = _diagnostic_px4_instance_generation;
		allocated.active_registration_mask = _active_registrations_mask;
		strncpy(allocated.registration_name, _registrations[free_registration_index].name,
			sizeof(allocated.registration_name) - 1);
		recordDiagnosticEvent(allocated);
	}

	return free_registration_index;
}

bool ExternalChecks::removeRegistration(int registration_id, int8_t nav_mode_id)
{
	if (registration_id < 0 || registration_id >= MAX_NUM_REGISTRATIONS) {
		return false;
	}

	if (registrationValid(registration_id)) {
		if (_registrations[registration_id].nav_mode_id == nav_mode_id) {
			_active_registrations_mask &= ~(1u << registration_id);
			DiagnosticEvent freed{};
			freed.timestamp = hrt_absolute_time();
			freed.disposition = DiagnosticDisposition::RegistrationFreed;
			freed.registration_id = static_cast<uint8_t>(registration_id);
			freed.registration_nav_mode_id = _registrations[registration_id].nav_mode_id;
			freed.registration_replaces_nav_state = _registrations[registration_id].replaces_nav_state;
			freed.registration_generation = _registrations[registration_id].generation;
			freed.px4_instance_generation = _diagnostic_px4_instance_generation;
			freed.active_registration_mask = _active_registrations_mask;
			strncpy(freed.registration_name, _registrations[registration_id].name,
				sizeof(freed.registration_name) - 1);
			recordDiagnosticEvent(freed);
			return true;
		}
	}

	PX4_ERR("trying to remove inactive external check");
	return false;
}

bool ExternalChecks::isUnresponsive(int registration_id)
{
	if (registration_id < 0 || registration_id >= MAX_NUM_REGISTRATIONS) {
		return false;
	}

	if (registrationValid(registration_id)) {
		return _registrations[registration_id].unresponsive;
	}

	return false;
}


void ExternalChecks::checkAndReport(const Context &context, Report &reporter)
{
	checkNonRegisteredModes(context, reporter);

	if (_active_registrations_mask == 0) {
		return;
	}

	NavModes unresponsive_modes{NavModes::None};

	for (int reg_idx = 0; reg_idx < MAX_NUM_REGISTRATIONS; ++reg_idx) {
		if (!registrationValid(reg_idx) || !_registrations[reg_idx].reply) {
			continue;
		}

		arming_check_reply_s &reply = *_registrations[reg_idx].reply;

		int8_t nav_mode_id = _registrations[reply.registration_id].nav_mode_id;

		if (_registrations[reply.registration_id].unresponsive) {

			if (nav_mode_id != -1) {
				unresponsive_modes = unresponsive_modes | reporter.getModeGroup(nav_mode_id);
				setOrClearRequirementBits(true, nav_mode_id, -1, reporter.failsafeFlags().mode_req_other);
			}

		} else {
			NavModes modes;

			// We distinguish between two cases:
			// - external navigation mode: in that case we set the single arming can_run bit for the mode
			// - generic external arming check: set all arming bits
			if (nav_mode_id == -1) {
				modes = NavModes::All;

			} else {
				modes = reporter.getModeGroup(nav_mode_id);

				int8_t replaces_nav_state = _registrations[reply.registration_id].replaces_nav_state;

				if (replaces_nav_state != -1) {
					modes = modes | reporter.getModeGroup(replaces_nav_state);
					// Also clear the arming bits for the replaced mode, as the user intention is always set to the
					// replaced mode.
					// We only have to clear the bits, as for the internal/replaced mode, the bits are not cleared yet.
				}

				if (!reply.can_arm_and_run) {
					setOrClearRequirementBits(true, nav_mode_id, replaces_nav_state, reporter.failsafeFlags().mode_req_other);
				}

				// Mode requirements
				// A replacement mode will also replace the mode requirements of the internal/replaced mode
				setOrClearRequirementBits(reply.mode_req_angular_velocity, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_angular_velocity);
				setOrClearRequirementBits(reply.mode_req_attitude, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_attitude);
				setOrClearRequirementBits(reply.mode_req_local_alt, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_local_alt);
				setOrClearRequirementBits(reply.mode_req_local_position, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_local_position);
				setOrClearRequirementBits(reply.mode_req_local_position_relaxed, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_local_position_relaxed);
				setOrClearRequirementBits(reply.mode_req_global_position, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_global_position);
				setOrClearRequirementBits(reply.mode_req_mission, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_mission);
				setOrClearRequirementBits(reply.mode_req_home_position, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_home_position);
				setOrClearRequirementBits(reply.mode_req_prevent_arming, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_prevent_arming);
				setOrClearRequirementBits(reply.mode_req_manual_control, nav_mode_id, replaces_nav_state,
							  reporter.failsafeFlags().mode_req_manual_control);
			}

			if (!reply.can_arm_and_run) {
				reporter.clearArmingBits(modes);
			}

			if (reply.health_component_index > 0) {
				reporter.setHealth((health_component_t)(1ull << reply.health_component_index),
						   reply.health_component_is_present, reply.health_component_warning,
						   reply.health_component_error);
			}

			for (int i = 0; i < reply.num_events; ++i) {
				// set the modes, which is the first argument
				memcpy(reply.events[i].arguments, &modes, sizeof(modes));

				reporter.addExternalEvent(reply.events[i], modes);
			}
		}
	}

	if (unresponsive_modes != NavModes::None) {
		DiagnosticEvent failure{};
		failure.timestamp = hrt_absolute_time();
		failure.disposition = DiagnosticDisposition::FailureReported;
		failure.expected_request_id = _current_request_id;
		failure.active_registration_mask = _active_registrations_mask;
		failure.received_mask_before = _reply_received_mask;
		failure.received_mask_after = _reply_received_mask;
		failure.no_reply_mask = _active_registrations_mask & ~_reply_received_mask;
		recordDiagnosticEvent(failure);
		dumpDiagnosticHistory();

		/* EVENT
		 * @description
		 * The application running the mode might have crashed or the CPU load is too high.
		 */
		reporter.armingCheckFailure(unresponsive_modes, health_component_t::system,
					    events::ID("check_external_modes_unresponsive"),
					    events::Log::Critical, "Mode is unresponsive");
	}

}

void ExternalChecks::update()
{
	if (_active_registrations_mask == 0) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	// Check for incoming replies
	arming_check_reply_s reply;
	int max_num_updates = arming_check_reply_s::ORB_QUEUE_LENGTH;

	while (_arming_check_reply_sub.update(&reply) && --max_num_updates >= 0) {
		const hrt_abstime reply_time = hrt_absolute_time();
		const uint32_t received_mask_before = _reply_received_mask;
		const bool registration_known = reply.registration_id < MAX_NUM_REGISTRATIONS && registrationValid(reply.registration_id);
		const bool current_request = _current_request_id == reply.request_id;
		const uint64_t reply_gap = registration_known && _last_accepted_reply_time[reply.registration_id] > 0
			? reply_time - _last_accepted_reply_time[reply.registration_id]
			: 0;

		DiagnosticEvent observed{};
		observed.timestamp = reply_time;
		observed.disposition = DiagnosticDisposition::ReplyObserved;
		observed.registration_id = reply.registration_id;
		observed.reply_request_id = reply.request_id;
		observed.expected_request_id = _current_request_id;
		observed.px4_instance_generation = _diagnostic_px4_instance_generation;
		observed.request_publish_sequence = _diagnostic_request_publish_sequence;
		observed.reply_timestamp = reply.timestamp;
		observed.health_component_index = reply.health_component_index;
		observed.num_events = reply.num_events;
		observed.active_registration_mask = _active_registrations_mask;
		observed.received_mask_before = received_mask_before;
		observed.received_mask_after = received_mask_before;
		observed.no_reply_mask = _active_registrations_mask & ~received_mask_before;
		observed.time_since_last_accepted_reply_us = reply_gap;
		observed.can_arm_and_run = reply.can_arm_and_run;
		if (registration_known) {
			observed.registration_generation = _registrations[reply.registration_id].generation;
			observed.registration_nav_mode_id = _registrations[reply.registration_id].nav_mode_id;
			observed.registration_replaces_nav_state = _registrations[reply.registration_id].replaces_nav_state;
			strncpy(observed.registration_name, _registrations[reply.registration_id].name,
				sizeof(observed.registration_name) - 1);
			observed.num_no_response = _registrations[reply.registration_id].num_no_response;
			observed.waiting_for_first_response = _registrations[reply.registration_id].waiting_for_first_response;
		}
		recordDiagnosticEvent(observed);

		if (registration_known && current_request) {
			const bool duplicate_current_reply = (received_mask_before & (1u << reply.registration_id)) != 0;
			_reply_received_mask |= 1u << reply.registration_id;
			_registrations[reply.registration_id].num_no_response = 0;
			_registrations[reply.registration_id].waiting_for_first_response = false;
			_last_accepted_reply_time[reply.registration_id] = reply_time;

			// Prevent toggling between unresponsive & responsive state
			if (_registrations[reply.registration_id].total_num_unresponsive <= 3) {
				_registrations[reply.registration_id].unresponsive = false;
			}

			if (_registrations[reply.registration_id].reply) {
				*_registrations[reply.registration_id].reply = reply;
			}

			DiagnosticEvent accepted = observed;
			accepted.disposition = DiagnosticDisposition::Accepted;
			accepted.received_mask_after = _reply_received_mask;
			accepted.no_reply_mask = _active_registrations_mask & ~_reply_received_mask;
			accepted.duplicate_current_reply = duplicate_current_reply;
			accepted.num_no_response = 0;
			accepted.waiting_for_first_response = false;
			recordDiagnosticEvent(accepted);

//			PX4_DEBUG("Registration id=%i: %i events", reply.registration_id, reply.num_events);
		} else {
			DiagnosticEvent rejected = observed;
			if (!registration_known) {
				rejected.disposition = DiagnosticDisposition::RejectedUnknownRegistration;
			} else if (!current_request) {
				rejected.disposition = DiagnosticDisposition::RejectedStaleRequestId;
			} else {
				rejected.disposition = DiagnosticDisposition::RejectedOther;
			}
			recordDiagnosticEvent(rejected);
		}
	}

	if (_last_update > 0) {
		if (_reply_received_mask == _active_registrations_mask) { // Got all responses
			// Nothing to do
		} else if (now > _last_update + REQUEST_TIMEOUT && !_had_timeout) { // Timeout
			_had_timeout = true;
			unsigned no_reply = _active_registrations_mask & ~_reply_received_mask;

			for (int i = 0; i < MAX_NUM_REGISTRATIONS; ++i) {
				if ((1u << i) & no_reply) {
					const int max_num_no_reply =
						_registrations[i].waiting_for_first_response ? NUM_NO_REPLY_UNTIL_UNRESPONSIVE_INIT : NUM_NO_REPLY_UNTIL_UNRESPONSIVE;

					if (!_registrations[i].unresponsive && ++_registrations[i].num_no_response > max_num_no_reply) {
						DiagnosticEvent unresponsive{};
						unresponsive.timestamp = now;
						unresponsive.disposition = DiagnosticDisposition::RegistrationUnresponsive;
						unresponsive.registration_id = static_cast<uint8_t>(i);
						unresponsive.expected_request_id = _current_request_id;
						unresponsive.px4_instance_generation = _diagnostic_px4_instance_generation;
						unresponsive.request_publish_sequence = _diagnostic_request_publish_sequence;
						unresponsive.registration_generation = _registrations[i].generation;
						unresponsive.registration_nav_mode_id = _registrations[i].nav_mode_id;
						unresponsive.registration_replaces_nav_state = _registrations[i].replaces_nav_state;
						strncpy(unresponsive.registration_name, _registrations[i].name,
							sizeof(unresponsive.registration_name) - 1);
						unresponsive.active_registration_mask = _active_registrations_mask;
						unresponsive.received_mask_before = _reply_received_mask;
						unresponsive.received_mask_after = _reply_received_mask;
						unresponsive.no_reply_mask = no_reply;
						unresponsive.time_since_last_accepted_reply_us =
							_last_accepted_reply_time[i] > 0 ? now - _last_accepted_reply_time[i] : 0;
						unresponsive.num_no_response = _registrations[i].num_no_response;
						unresponsive.waiting_for_first_response = _registrations[i].waiting_for_first_response;
						recordDiagnosticEvent(unresponsive);

						// Clear immediately if not a mode
						if (_registrations[i].nav_mode_id == -1) {
							removeRegistration(i, -1);
							PX4_WARN("No response from %i, removing", i);

						} else {
							_registrations[i].unresponsive = true;

							if (_registrations[i].total_num_unresponsive < 100) {
								++_registrations[i].total_num_unresponsive;
							}

							PX4_WARN("No response from %i, flagging unresponsive", i);
						}
					}
				}
			}
		}
	}

	// Start a new request?
	if (now > _last_update + UPDATE_INTERVAL) {
		_reply_received_mask = 0;
		_last_update = now;
		_had_timeout = false;

		// Request the state from all registered components
		arming_check_request_s request{};
		request.request_id = ++_current_request_id;
		request.timestamp = hrt_absolute_time();
		DiagnosticEvent request_event{};
		request_event.timestamp = request.timestamp;
		request_event.disposition = DiagnosticDisposition::RequestPublished;
		request_event.reply_request_id = request.request_id;
		request_event.expected_request_id = request.request_id;
		request_event.request_publish_sequence = ++_diagnostic_request_publish_sequence;
		request_event.px4_instance_generation = _diagnostic_px4_instance_generation;
		request_event.active_registration_mask = _active_registrations_mask;
		request_event.received_mask_before = _reply_received_mask;
		request_event.received_mask_after = _reply_received_mask;
		request_event.no_reply_mask = _active_registrations_mask;
		recordDiagnosticEvent(request_event);
		_arming_check_request_pub.publish(request);
	}
}

void ExternalChecks::setExternalNavStates(uint8_t first_external_nav_state, uint8_t last_external_nav_state)
{
	_first_external_nav_state = first_external_nav_state;
	_last_external_nav_state = last_external_nav_state;
}

void ExternalChecks::checkNonRegisteredModes(const Context &context, Report &reporter) const
{
	// Clear the arming bits for all non-registered external modes.
	// But only report if one of them is selected, so we don't need to generate the extra event in most cases.
	bool report_mode_not_available = false;

	for (uint8_t external_nav_state = _first_external_nav_state; external_nav_state <= _last_external_nav_state;
	     ++external_nav_state) {
		bool found = false;

		for (int reg_idx = 0; reg_idx < MAX_NUM_REGISTRATIONS; ++reg_idx) {
			if (registrationValid(reg_idx) && _registrations[reg_idx].nav_mode_id == external_nav_state) {
				found = true;
				break;
			}
		}

		if (!found) {
			if (external_nav_state == context.status().nav_state_user_intention) {
				report_mode_not_available = true;
			}

			reporter.clearArmingBits(reporter.getModeGroup(external_nav_state));
			setOrClearRequirementBits(true, external_nav_state, -1, reporter.failsafeFlags().mode_req_other);
		}
	}

	if (report_mode_not_available) {
		/* EVENT
		 * @description
		 * The application running the mode is not started.
		 */
		reporter.armingCheckFailure(reporter.getModeGroup(context.status().nav_state_user_intention),
					    health_component_t::system,
					    events::ID("check_external_modes_unavailable"),
					    events::Log::Error, "Mode is not registered");
	}
}
