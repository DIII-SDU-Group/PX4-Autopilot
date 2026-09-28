/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cstddef>
#include <cstdint>

#include "../Common.hpp"
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <px4_platform_common/module_params.h>

class ExternalChecks : public HealthAndArmingCheckBase
{
public:
	static constexpr int MAX_NUM_REGISTRATIONS = 8;

	ExternalChecks();
	~ExternalChecks() = default;

	void setExternalNavStates(uint8_t first_external_nav_state, uint8_t last_external_nav_state);

	void checkAndReport(const Context &context, Report &reporter) override;

	bool hasFreeRegistrations() const { return _active_registrations_mask != (1u << MAX_NUM_REGISTRATIONS) - 1; }
	/**
	 * Add registration
	 * @param nav_mode_id associated mode, -1 if none
	 * @param replaces_nav_state replaced mode, -1 if none
	 * @return registration id, or -1
	 */
	int addRegistration(int8_t nav_mode_id, int8_t replaces_nav_state,
					const char *registration_name = nullptr);
	bool removeRegistration(int registration_id, int8_t nav_mode_id);
	void update();

	bool isUnresponsive(int registration_id);
	bool allowUpdateWhileArmed() const { return _param_com_mode_arm_chk.get(); }
private:
	// The III split-host path runs the XRCE agent on the companion Pi. A
	// five-mode reply burst is emitted by ROS in under 25 ms, but can take up to
	// about two seconds to traverse the agent and return to PX4. Keep the request
	// id valid for that distributed path. This checks registration health only;
	// active-mode setpoint and flight-control failsafes remain independent.
	static constexpr hrt_abstime REQUEST_TIMEOUT = 2500_ms;
	static constexpr hrt_abstime UPDATE_INTERVAL = 3_s;
	static_assert(REQUEST_TIMEOUT < UPDATE_INTERVAL, "keep timeout < update interval");
	static constexpr int NUM_NO_REPLY_UNTIL_UNRESPONSIVE = 3; ///< Mode timeout = this value * UPDATE_INTERVAL
	/// Timeout directly after registering (in some cases ROS can take a while until the subscription gets the first
	/// sample, around 800ms was observed)
	static constexpr int NUM_NO_REPLY_UNTIL_UNRESPONSIVE_INIT = 10;

	void checkNonRegisteredModes(const Context &context, Report &reporter) const;

	enum class DiagnosticDisposition : uint8_t {
		RequestPublished,
		RegistrationAllocated,
		RegistrationFreed,
		ReplyObserved,
		Accepted,
		RejectedStaleRequestId,
		RejectedUnknownRegistration,
		RejectedOther,
		RegistrationUnresponsive,
		FailureReported,
	};

	struct DiagnosticEvent {
		hrt_abstime timestamp{0};
		uint32_t sequence{0};
		DiagnosticDisposition disposition{DiagnosticDisposition::ReplyObserved};
		uint8_t registration_id{UINT8_MAX};
		uint8_t reply_request_id{0};
		uint8_t expected_request_id{0};
		uint8_t health_component_index{0};
		uint8_t num_events{0};
		uint8_t num_no_response{0};
		int8_t registration_nav_mode_id{-1};
		int8_t registration_replaces_nav_state{-1};
		uint64_t reply_timestamp{0};
		uint32_t request_publish_sequence{0};
		uint32_t px4_instance_generation{0};
		uint32_t registration_generation{0};
		uint32_t active_registration_mask{0};
		uint32_t received_mask_before{0};
		uint32_t received_mask_after{0};
		uint32_t no_reply_mask{0};
		uint64_t time_since_last_accepted_reply_us{0};
		bool can_arm_and_run{false};
		bool duplicate_current_reply{false};
		bool waiting_for_first_response{false};
		char registration_name[sizeof(register_ext_component_request_s::name)] {};
	};

	static constexpr size_t DIAGNOSTIC_HISTORY_SIZE = 128;

	static const char *diagnosticDispositionName(DiagnosticDisposition disposition);
	void recordDiagnosticEvent(const DiagnosticEvent &event);
	void dumpDiagnosticHistory();

	DiagnosticEvent _diagnostic_history[DIAGNOSTIC_HISTORY_SIZE] {};
	uint32_t _diagnostic_next_sequence{0};
	uint32_t _diagnostic_history_count{0};
	uint32_t _diagnostic_history_next{0};
	bool _diagnostic_failure_dumped{false};

	bool registrationValid(int reg_idx) const { return ((1u << reg_idx) & _active_registrations_mask) != 0; }

	struct Registration {
		~Registration() { delete reply; }

		int8_t nav_mode_id{-1}; ///< associated mode, -1 if none
		int8_t replaces_nav_state{-1};
		char name[sizeof(register_ext_component_request_s::name)] {};
		uint32_t generation{0};

		bool waiting_for_first_response{true};
		uint8_t num_no_response{0};
		bool unresponsive{false};
		uint8_t total_num_unresponsive{0};
		arming_check_reply_s *reply{nullptr};
	};

	unsigned _active_registrations_mask{0};
	Registration _registrations[MAX_NUM_REGISTRATIONS] {};
	hrt_abstime _last_accepted_reply_time[MAX_NUM_REGISTRATIONS] {};

	uint8_t _first_external_nav_state = vehicle_status_s::NAVIGATION_STATE_MAX;
	uint8_t _last_external_nav_state = vehicle_status_s::NAVIGATION_STATE_MAX;
	uint32_t _diagnostic_px4_instance_generation{0};
	uint32_t _diagnostic_request_publish_sequence{0};

	// Current requests (async updates)
	hrt_abstime _last_update{0};
	unsigned _reply_received_mask{0};
	bool _had_timeout{false};

	uint8_t _current_request_id{0};

	uORB::Subscription _arming_check_reply_sub{ORB_ID(arming_check_reply)};

	uORB::Publication<arming_check_request_s> _arming_check_request_pub{ORB_ID(arming_check_request)};
	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamBool<px4::params::COM_MODE_ARM_CHK>) _param_com_mode_arm_chk
				       );
};
