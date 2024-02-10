#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MsgKind {
    Data,
    Event,
}

#[repr(transparent)]
#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BdAddr([u8; 6]);

impl BdAddr {
    pub fn new(val: [u8; 6]) -> Self {
        Self(val)
    }

    pub fn to_raw(self) -> [u8; 6] {
        self.0
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ConnHandle(u16);

impl ConnHandle {
    pub fn new(val: u16) -> Self {
        Self(val)
    }

    pub fn to_raw(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EventMask([u8; 8]);

impl EventMask {
    pub fn new(val: [u8; 8]) -> Self {
        Self(val)
    }

    pub fn to_raw(self) -> [u8; 8] {
        self.0
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EventMaskPage2([u8; 8]);

impl EventMaskPage2 {
    pub fn new(val: [u8; 8]) -> Self {
        Self(val)
    }

    pub fn to_raw(self) -> [u8; 8] {
        self.0
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LeEventMask([u8; 8]);

impl LeEventMask {
    pub fn new(val: [u8; 8]) -> Self {
        Self(val)
    }

    pub fn to_raw(self) -> [u8; 8] {
        self.0
    }
}

macro_rules! events {
    (
        $(
            $kind:ident {
                $(
                    ($val:expr, $setter:ident, $getter:ident);
                )+
            }
        )+
    ) => {
        $(
            impl $kind {
                $(
                    pub fn $getter(&self) -> bool {
                        const BYTE: usize = $val / 8;
                        const BIT: usize = $val % 8;
                        (self.0[BYTE] & (1 << BIT)) != 0
                    }

                    pub fn $setter(&mut self, enabled: bool) -> &mut Self {
                        const BYTE: usize = $val / 8;
                        const BIT: usize = $val % 8;
                        self.0[BYTE] = (self.0[BYTE] & !(1 << BIT)) | (u8::from(enabled) << BIT);
                        self
                    }
                )+
            }
        )+
    }
}

events! {
    EventMask {
        (0, enable_inquiry_complete, is_inquiry_complete_enabled);
        (1, enable_inquiry_result, is_inquiry_result_enabled);
        (2, enable_connection_complete, is_connection_complete_enabled);
        (3, enable_connection_request, is_connection_request_enabled);
        (4, enable_disconnection_complete, is_disconnection_complete_enabled);
        (5, enable_authentication_complete, is_authentication_complete_enabled);
        (6, enable_remote_name_request_complete, is_remote_name_request_complete_enabled);
        (7, enable_encryption_change_v1, is_encryption_change_v1_enabled);
        (8, enable_change_connection_link_key_complete, is_change_connection_link_key_complete_enabled);
        (9, enable_link_key_type_changed, is_link_key_type_changed_enabled);
        (10, enable_read_remote_supported_features_complete, supports_read_remote_features_complete_enabled);
        (11, enable_read_remote_version_information_complete, is_read_remote_version_information_complete_enabled);
        (12, enable_qos_setup_complete, is_qos_setup_complete_enabled);
        (15, enable_hardware_error, is_hardware_error_enabled);
        (16, enable_flush_occurred, is_flush_occurred_enabled);
        (17, enable_role_change, is_role_change_enabled);
        (19, enable_mode_change, is_mode_change_enabled);
        (20, enable_return_link_keys, is_return_link_keys_enabled);
        (21, enable_pin_code_request, is_pin_code_request_enabled);
        (22, enable_link_key_request, is_link_key_request_enabled);
        (23, enable_link_key_notification, is_link_key_notification_enabled);
        (24, enable_loopback_command, is_loopback_command_enabled);
        (25, enable_data_buffer_overflow, is_data_buffer_overflow_enabled);
        (26, enable_max_slots_change, is_max_slots_change_enabled);
        (27, enable_read_clock_offset_complete, is_read_clock_offset_complete_enabled);
        (28, enable_connection_packet_type_changed, is_connection_packet_type_changed_enabled);
        (29, enable_qos_violation, is_qos_violation_enabled);
        (31, enable_page_scan_repetition_mode_change, is_page_scan_repetition_mode_change_enabled);
        (32, enable_flow_specification_complete, is_flow_specification_complete_enabled);
        (33, enable_inquiry_result_with_rssi, is_inquiry_result_with_rssi_enabled);
        (34, enable_read_remote_extended_features_complete, is_read_remote_extended_features_complete_enabled);
        (43, enable_synchronous_connection_complete, is_synchronous_connection_complete_enabled);
        (44, enable_synchronous_connection_changed, is_synchronous_connection_changed_enabled);
        (45, enable_sniff_subrating, is_sniff_subrating_enabled);
        (46, enable_extended_inquiry_result, is_extended_inquiry_result_enabled);
        (47, enable_encryption_key_refresh_complete, is_encryption_key_refresh_complete_enabled);
        (48, enable_io_capability_request, is_io_capability_request_enabled);
        (49, enable_io_capability_response, is_io_capability_response_enabled);
        (50, enable_user_confirmation_request, is_user_confirmation_request_enabled);
        (51, enable_user_passkey_request, is_user_passkey_request_enabled);
        (52, enable_remote_oob_data_request, is_remote_oob_data_request_enabled);
        (53, enable_simple_pairing_complete, is_simple_pairing_complete_enabled);
        (55, enable_link_supervision_timeout_changed, is_link_supervision_timeout_changed_enabled);
        (56, enable_enhanced_flush_complete, is_enhanced_flush_complete_enabled);
        (58, enable_user_passkey_notification, is_user_passkey_notification_enabled);
        (59, enable_keypress_notification, is_keypress_notification_enabled);
        (60, enable_remote_host_supported_features_notification, supports_remote_host_features_notification_enabled);
        (61, enable_le_meta, is_le_meta_enabled);
    }

    EventMaskPage2 {
        (8, enable_number_of_completed_data_blocks, is_number_of_completed_data_blocks_enabled);
        (14, enable_triggered_clock_capture, is_triggered_clock_capture_enabled);
        (15, enable_synchronization_train_complete, is_synchronization_train_complete_enabled);
        (16, enable_synchronization_train_received, is_synchronization_train_received_enabled);
        (17, enable_connectionless_peripheral_broadcast_receive, is_connectionless_peripheral_broadcast_receive_enabled);
        (18, enable_connectionless_peripheral_broadcast_timeout, is_connectionless_peripheral_broadcast_timeout_enabled);
        (19, enable_truncated_page_complete, is_truncated_page_complete_enabled);
        (20, enable_peripheral_page_response_timeout, is_peripheral_page_response_timeout_enabled);
        (21, enable_connectionless_peripheral_broadcast_channel_map_change, is_connectionless_peripheral_broadcast_channel_map_change_enabled);
        (22, enable_inquiry_response_notification, is_inquiry_response_notification_enabled);
        (23, enable_authenticated_payload_timeout_expired, is_authenticated_payload_timeout_expired_enabled);
        (24, enable_sam_status_change, is_sam_status_change_enabled);
        (25, enable_encryption_change_v2, is_encryption_change_v2_enabled);
    }

    LeEventMask {
        (0, enable_le_connection_complete, is_le_connection_complete_enabled);
        (1, enable_le_advertising_report, is_le_advertising_report_enabled);
        (2, enable_le_connection_update_complete, is_le_connection_update_complete_enabled);
        (3, enable_le_read_remote_features_complete, is_le_read_remote_features_complete_enabled);
        (4, enable_le_long_term_key_request, is_le_long_term_key_request_enabled);
        (5, enable_le_remote_connection_parameter_request, is_le_remote_connection_parameter_request_enabled);
        (6, enable_le_data_length_change, is_le_data_length_change_enabled);
        (7, enable_le_read_local_p256_public_key_complete, is_le_read_local_p256_public_key_complete_enabled);
        (8, enable_le_generate_dhkey_complete, is_le_generate_dhkey_complete_enabled);
        (9, enable_le_enhanced_connection_complete, is_le_enhanced_connection_complete_enabled);
        (10, enable_le_directed_advertising_report, is_le_directed_advertising_report_enabled);
        (11, enable_le_phy_update_complete, is_le_phy_update_complete_enabled);
        (12, enable_le_extended_advertising_report, is_le_extended_advertising_report_enabled);
        (13, enable_le_periodic_advertising_sync_established, is_le_periodic_advertising_sync_established_enabled);
        (14, enable_le_periodic_advertising_report, is_le_periodic_advertising_report_enabled);
        (15, enable_le_periodic_advertising_sync_lost, is_le_periodic_advertising_sync_lost_enabled);
        (16, enable_le_scan_timeout, is_le_scan_timeout_enabled);
        (17, enable_le_advertising_set_terminated, is_le_advertising_set_terminated_enabled);
        (18, enable_le_scan_request_received, is_le_scan_request_received_enabled);
        (19, enable_le_channel_selection_algorithm, is_le_channel_selection_algorithm_enabled);
        (20, enable_le_connectionless_iq_report, is_le_connectionless_iq_report_enabled);
        (21, enable_le_connection_iq_report, is_le_connection_iq_report_enabled);
        (22, enable_le_cte_request_failed, is_le_cte_request_failed_enabled);
        (23, enable_le_periodic_advertising_sync_transfer_received, is_le_periodic_advertising_sync_transfer_received_enabled);
        (24, enable_le_cis_established, is_le_cis_established_enabled);
        (25, enable_le_cis_request, is_le_cis_request_enabled);
        (26, enable_le_create_big_complete, is_le_create_big_complete_enabled);
        (27, enable_le_terminate_big_complete, is_le_terminate_big_complete_enabled);
        (28, enable_le_big_sync_established, is_le_big_sync_established_enabled);
        (29, enable_le_big_sync_lost, is_le_big_sync_lost_enabled);
        (30, enable_le_request_peer_sca_complete, is_le_request_peer_sca_complete_enabled);
        (31, enable_le_path_loss_threshold, is_le_path_loss_threshold_enabled);
        (32, enable_le_transmit_power_reporting, is_le_transmit_power_reporting_enabled);
        (33, enable_le_biginfo_advertising_report, is_le_biginfo_advertising_report_enabled);
        (34, enable_le_subrate_change, is_le_subrate_change_enabled);
    }
}

#[repr(transparent)]
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CmdMask([u8; 64]);

impl CmdMask {
    pub fn new(val: [u8; 64]) -> Self {
        Self(val)
    }

    pub fn as_raw(&self) -> &[u8; 64] {
        &self.0
    }
}

macro_rules! commands {
    (
        $(
            $octet:expr => {
                $(
                    ($bit:expr, $getter:ident);
                )+
            }
        )+
    ) => {
        impl CmdMask {
            $(
                $(
                    pub fn $getter(&self) -> bool {
                        (self.0[$octet] & (1 << $bit)) != 0
                    }
                )+
            )+
        }
    }
}

commands! {
    0 => {
        (0, inquiry);
        (1, inquiry_cancel);
        (2, periodic_inquiry_mode);
        (3, exit_periodic_inquiry_mode);
        (4, create_connection);
        (5, disconnect);
        (7, create_connection_cancel);
    }
    1 => {
        (0, accept_connection_request);
        (1, reject_connection_request);
        (2, link_key_request_reply);
        (3, link_key_request_negative_reply);
        (4, pin_code_request_reply);
        (5, pin_code_request_negative_reply);
        (6, change_connection_packet_type);
        (7, authentication_requested);
    }
    2 => {
        (0, set_connection_encryption);
        (1, change_connection_link_key);
        (2, link_key_selection);
        (3, remote_name_request);
        (4, remote_name_request_cancel);
        (5, read_remote_supported_features);
        (6, read_remote_extended_features);
        (7, read_remote_version_information);
    }
    3 => {
        (0, read_clock_offset);
        (1, read_lmp_handle);
    }
    4 => {
        (1, hold_mode);
        (2, sniff_mode);
        (3, exit_sniff_mode);
        (6, qos_setup);
        (7, role_discovery);
    }
    5 => {
        (0, switch_role);
        (1, read_link_policy_settings);
        (2, write_link_policy_settings);
        (3, read_default_link_policy_settings);
        (4, write_default_link_policy_settings);
        (5, flow_specification);
        (6, set_event_mask);
        (7, reset);
    }
    6 => {
        (0, set_event_filter);
        (1, flush);
        (2, read_pin_type);
        (3, write_pin_type);
        (5, read_stored_link_key);
        (6, write_stored_link_key);
        (7, delete_stored_link_key);
    }
    7 => {
        (0, write_local_name);
        (1, read_local_name);
        (2, read_connection_accept_timeout);
        (3, write_connection_accept_timeout);
        (4, read_page_timeout);
        (5, write_page_timeout);
        (6, read_scan_enable);
        (7, write_scan_enable);
    }
    8 => {
        (0, read_page_scan_activity);
        (1, write_page_scan_activity);
        (2, read_inquiry_scan_activity);
        (3, write_inquiry_scan_activity);
        (4, read_authentication_enable);
        (5, write_authentication_enable);
    }
    9 => {
        (0, read_class_of_device);
        (1, write_class_of_device);
        (2, read_voice_setting);
        (3, write_voice_setting);
        (4, read_automatic_flush_timeout);
        (5, write_automatic_flush_timeout);
        (6, read_num_broadcast_retransmissions);
        (7, write_num_broadcast_retransmissions);
    }
    10 => {
        (0, read_hold_mode_activity);
        (1, write_hold_mode_activity);
        (2, read_transmit_power_level);
        (3, read_synchronous_flow_control_enable);
        (4, write_synchronous_flow_control_enable);
        (5, set_controller_to_host_flow_control);
        (6, host_buffer_size);
        (7, host_number_of_completed_packets);
    }
    11 => {
        (0, read_link_supervision_timeout);
        (1, write_link_supervision_timeout);
        (2, read_number_of_supported_iac);
        (3, read_current_iac_lap);
        (4, write_current_iac_lap);
    }
    12 => {
        (1, set_afh_host_channel_classification);
        (4, read_inquiry_scan_type);
        (5, write_inquiry_scan_type);
        (6, read_inquiry_mode);
        (7, write_inquiry_mode);
    }
    13 => {
        (0, read_page_scan_type);
        (1, write_page_scan_type);
        (2, read_afh_channel_assessment_mode);
        (3, write_afh_channel_assessment_mode);
    }
    14 => {
        (3, read_local_version_information);
        (5, read_local_supported_features);
        (6, read_local_extended_features);
        (7, read_buffer_size);
    }
    15 => {
        (1, read_bd_addr);
        (2, read_failed_contact_counter);
        (3, reset_failed_contact_counter);
        (4, read_link_quality);
        (5, read_rssi);
        (6, read_afh_channel_map);
        (7, read_clock);
    }
    16 => {
        (0, read_loopback_mode);
        (1, write_loopback_mode);
        (2, enable_device_under_test_mode);
        (3, setup_synchronous_connection_request);
        (4, accept_synchronous_connection_request);
        (5, reject_synchronous_connection_request);
    }
    17 => {
        (0, read_extended_inquiry_response);
        (1, write_extended_inquiry_response);
        (2, refresh_encryption_key);
        (4, sniff_subrating);
        (5, read_simple_pairing_mode);
        (6, write_simple_pairing_mode);
        (7, read_local_oob_data);
    }
    18 => {
        (0, read_inquiry_response_transmit_power_level);
        (1, write_inquiry_transmit_power_level);
        (2, read_default_erroneous_data_reporting);
        (3, write_default_erroneous_data_reporting);
        (7, io_capability_request_reply);
    }
    19 => {
        (0, user_confirmation_request_reply);
        (1, user_confirmation_request_negative_reply);
        (2, user_passkey_request_reply);
        (3, user_passkey_request_negative_reply);
        (4, remote_oob_data_request_reply);
        (5, write_simple_pairing_debug_mode);
        (6, enhanced_flush);
        (7, remote_oob_data_request_negative_reply);
    }
    20 => {
        (2, send_keypress_notification);
        (3, io_capability_request_negative_reply);
        (4, read_encryption_key_size);
    }
    22 => {
        (2, set_event_mask_page_2);
    }
    23 => {
        (0, read_flow_control_mode);
        (1, write_flow_control_mode);
        (2, read_data_block_size);
    }
    24 => {
        (0, read_enhanced_transmit_power_level);
        (5, read_le_host_support);
        (6, write_le_host_support);
    }
    25 => {
        (0, le_set_event_mask);
        (1, le_read_buffer_size_v1);
        (2, le_read_local_supported_features);
        (4, le_set_random_address);
        (5, le_set_advertising_parameters);
        (6, le_read_advertising_physical_channel_tx_power);
        (7, le_set_advertising_data);
    }
    26 => {
        (0, le_set_scan_response_data);
        (1, le_set_advertising_enable);
        (2, le_set_scan_parameters);
        (3, le_set_scan_enable);
        (4, le_create_connection);
        (5, le_create_connection_cancel);
        (6, le_read_filter_accept_list_size);
        (7, le_clear_filter_accept_list);
    }
    27 => {
        (0, le_add_device_to_filter_accept_list);
        (1, le_remove_device_from_filter_accept_list);
        (2, le_connection_update);
        (3, le_set_host_channel_classification);
        (4, le_read_channel_map);
        (5, le_read_remote_features);
        (6, le_encrypt);
        (7, le_rand);
    }
    28 => {
        (0, le_enable_encryption);
        (1, le_long_term_key_request_reply);
        (2, le_long_term_key_request_negative_reply);
        (3, le_read_supported_states);
        (4, le_receiver_test_v1);
        (5, le_transmitter_test_v1);
        (6, le_test_end);
    }
    29 => {
        (3, enhanced_setup_synchronous_connection);
        (4, enhanced_accept_synchronous_connection);
        (5, read_local_supported_codecs);
        (6, set_mws_channel_parameters);
        (7, set_external_frame_configuration);
    }
    30 => {
        (0, set_mws_signaling);
        (1, set_mws_transport_layer);
        (2, set_mws_scan_frequency_table);
        (3, get_mws_transport_layer_configuration);
        (4, set_mws_pattern_configuration);
        (5, set_triggered_clock_capture);
        (6, truncated_page);
        (7, truncated_page_cancel);
    }
    31 => {
        (0, set_connectionless_peripheral_broadcast);
        (1, set_connectionless_peripheral_broadcast_receive);
        (2, start_synchronization_train);
        (3, receive_synchronization_train);
        (4, set_reserved_lt_addr);
        (5, delete_reserved_lt_addr);
        (6, set_connectionless_peripheral_broadcast_data);
        (7, read_synchronization_train_parameters);
    }
    32 => {
        (0, write_synchronization_train_parameters);
        (1, remote_oob_extended_data_request_reply);
        (2, read_secure_connections_host_support);
        (3, write_secure_connections_host_support);
        (4, read_authenticated_payload_timeout);
        (5, write_authenticated_payload_timeout);
        (6, read_local_oob_extended_data);
        (7, write_secure_connections_test_mode);
    }
    33 => {
        (0, read_extended_page_timeout);
        (1, write_extended_page_timeout);
        (2, read_extended_inquiry_length);
        (3, write_extended_inquiry_length);
        (4, le_remote_connection_parameter_request_reply);
        (5, le_remote_connection_parameter_request_negative_reply);
        (6, le_set_data_length);
        (7, le_read_suggested_default_data_length);
    }
    34 => {
        (0, le_write_suggested_default_data_length);
        (1, le_read_local_p256_public_key);
        (2, le_generate_dhkey_v1);
        (3, le_add_device_to_resolving_list);
        (4, le_remove_device_from_resolving_list);
        (5, le_clear_resolving_list);
        (6, le_read_resolving_list_size);
        (7, le_read_peer_resolvable_address);
    }
    35 => {
        (0, le_read_local_resolvable_address);
        (1, le_set_address_resolution_enable);
        (2, le_set_resolvable_private_address_timeout);
        (3, le_read_maximum_data_length);
        (4, le_read_phy);
        (5, le_set_default_phy);
        (6, le_set_phy);
        (7, le_receiver_test_v2);
    }
    36 => {
        (0, le_transmitter_test_v2);
        (1, le_set_advertising_set_random_address);
        (2, le_set_extended_advertising_parameters);
        (3, le_set_extended_advertising_data);
        (4, le_set_extended_scan_response_data);
        (5, le_set_extended_advertising_enable);
        (6, le_read_maximum_advertising_data_length);
        (7, le_read_number_of_supported_advertising_sets);
    }
    37 => {
        (0, le_remove_advertising_set);
        (1, le_clear_advertising_sets);
        (2, le_set_periodic_advertising_parameters);
        (3, le_set_periodic_advertising_data);
        (4, le_set_periodic_advertising_enable);
        (5, le_set_extended_scan_parameters);
        (6, le_set_extended_scan_enable);
        (7, le_extended_create_connection);
    }
    38 => {
        (0, le_periodic_advertising_create_sync);
        (1, le_periodic_advertising_create_sync_cancel);
        (2, le_periodic_advertising_terminate_sync);
        (3, le_add_device_to_periodic_advertiser_list);
        (4, le_remove_device_from_periodic_advertiser_list);
        (5, le_clear_periodic_advertiser_list);
        (6, le_read_periodic_advertiser_list_size);
        (7, le_read_transmit_power);
    }
    39 => {
        (0, le_read_rf_path_compensation);
        (1, le_write_rf_path_compensation);
        (2, le_set_privacy_mode);
        (3, le_receiver_test_v3);
        (4, le_transmitter_test_v3);
        (5, le_set_connectionless_cte_transmit_parameters);
        (6, le_set_connectionless_cte_transmit_enable);
        (7, le_set_connectionless_iq_sampling_enable);
    }
    40 => {
        (0, le_set_connection_cte_receive_parameters);
        (1, le_set_connection_cte_transmit_parameters);
        (2, le_connection_cte_request_enable);
        (3, le_connection_cte_response_enable);
        (4, le_read_antenna_information);
        (5, le_set_periodic_advertising_receive_enable);
        (6, le_periodic_advertising_sync_transfer);
        (7, le_periodic_advertising_set_info_transfer);
    }
    41 => {
        (0, le_set_periodic_advertising_sync_transfer_parameters);
        (1, le_set_default_periodic_advertising_sync_transfer_parameters);
        (2, le_generate_dhkey_v2);
        (3, read_local_simple_pairing_options);
        (4, le_modify_sleep_clock_accuracy);
        (5, le_read_buffer_size_v2);
        (6, le_read_iso_tx_sync);
        (7, le_set_cig_parameters);
    }
    42 => {
        (0, le_set_cig_parameters_test);
        (1, le_create_cis);
        (2, le_remove_cig);
        (3, le_accept_cis_request);
        (4, le_reject_cis_request);
        (5, le_create_big);
        (6, le_create_big_test);
        (7, le_terminate_big);
    }
    43 => {
        (0, le_big_create_sync);
        (1, le_big_terminate_sync);
        (2, le_request_peer_sca);
        (3, le_setup_iso_data_path);
        (4, le_remove_iso_data_path);
        (5, le_iso_transmit_test);
        (6, le_iso_receive_test);
        (7, le_iso_read_test_counters);
    }
    44 => {
        (0, le_iso_test_end);
        (1, le_set_host_feature);
        (2, le_read_iso_link_quality);
        (3, le_enhanced_read_transmit_power_level);
        (4, le_read_remote_transmit_power_level);
        (5, le_set_path_loss_reporting_parameters);
        (6, le_set_path_loss_reporting_enable);
        (7, le_set_transmit_power_reporting_enable);
    }
    45 => {
        (0, le_transmitter_test_v4);
        (1, set_ecosystem_base_interval);
        (2, read_local_supported_codecs_v2);
        (3, read_local_supported_codec_capabilities);
        (4, read_local_supported_controller_delay);
        (5, configure_data_path);
        (6, le_set_data_related_address_changes);
        (7, set_min_encryption_key_size);
    }
    46 => {
        (0, le_set_default_subrate);
        (1, le_subrate_request);
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LmpFeatureMask([u8; 8]);

impl LmpFeatureMask {
    pub fn new(val: [u8; 8]) -> Self {
        Self(val)
    }

    pub fn to_raw(self) -> [u8; 8] {
        self.0
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LeFeatureMask([u8; 8]);

impl LeFeatureMask {
    pub fn new(val: [u8; 8]) -> Self {
        Self(val)
    }

    pub fn to_raw(self) -> [u8; 8] {
        self.0
    }
}

macro_rules! lmp_features {
    (
        $(
            $page:ident {
                $(
                    ($num:expr, $getter:ident);
                )+
            }
        )+
    ) => {
        $(
            impl $page {
                $(
                    pub fn $getter(&self) -> bool {
                        const BYTE: usize = $num / 8;
                        const BIT: usize = $num % 8;
                        (self.0[BYTE] & (1 << BIT)) != 0
                    }
                )+
            }
        )+
    }
}

lmp_features! {
    LmpFeatureMask {
        (0, supports_3_slot_packets);
        (1, supports_5_slot_packets);
        (2, supports_encryption);
        (3, supports_slot_offset);
        (4, supports_timing_accuracy);
        (5, supports_role_switch);
        (6, supports_hold_mode);
        (7, supports_sniff_mode);
        (9, supports_power_control_requests);
        (10, supports_cqddr);
        (11, supports_sco_link);
        (12, supports_hv2_packets);
        (13, supports_hv3_packets);
        (14, supports_mu_law_log_synchronous_data);
        (15, supports_a_law_log_synchronous_data);
        (16, supports_cvsd_synchronous_data);
        (17, supports_paging_parameter_negotiation);
        (18, supports_power_control);
        (19, supports_transparent_synchronous_data);
        (20, supports_flow_control_lag_lsb);
        (21, supports_flow_control_lag_middle_bit);
        (22, supports_flow_control_lag_msb);
        (23, supports_broadcast_encryption);
        (25, supports_enhanced_data_rate_acl_2mbps_mode);
        (26, supports_enhanced_data_rate_acl_3mbps_mode);
        (27, supports_enhanced_inquiry_scan);
        (28, supports_interlaced_inquiry_scan);
        (29, supports_interlaced_page_scan);
        (30, supports_rssi_with_inquiry_results);
        (31, supports_extended_sco_link);
        (32, supports_ev4_packets);
        (33, supports_ev5_packets);
        (35, supports_afh_capable_peripheral);
        (36, supports_afh_classification_peripheral);
        (37, supports_br_edr_not);
        (38, supports_le);
        (39, supports_3_slot_enhanced_data_rate_acl_packets);
        (40, supports_5_slot_enhanced_data_rate_acl_packets);
        (41, supports_sniff_subrating);
        (42, supports_pause_encryption);
        (43, supports_afh_capable_central);
        (44, supports_afh_classification_central);
        (45, supports_enhanced_data_rate_esco_2mbps_mode);
        (46, supports_enhanced_data_rate_esco_3mbps_mode);
        (47, supports_3_slot_enhanced_data_rate_esco_packets);
        (48, supports_extended_inquiry_response);
        (49, supports_simultaneous_le_and_br_edr_to_same_devi);
        (51, supports_secure_simple_pairing);
        (52, supports_encapsulated_pdu);
        (53, supports_erroneous_data_reporting);
        (54, supports_non_flushable_packet_boundary_flag);
        (56, supports_hci_link_supervision_timeout_changed_event);
        (57, supports_variable_inquiry_tx_power_level);
        (58, supports_enhanced_power_control);
        (63, supports_extended_features);
    }

    LmpFeatureMask {
        (0, supports_le_encryption);
        (1, supports_connection_parameters_request_procedure);
        (2, supports_extended_reject_indication);
        (3, supports_peripheral_initiated_features_exchange);
        (4, supports_le_ping);
        (5, supports_le_data_packet_length_extension);
        (6, supports_ll_privacy);
        (7, supports_extended_scanner_filter_policies);
        (8, supports_le_2m_phy);
        (9, supports_stable_modulation_index_tx);
        (10, supports_stable_modulation_index_rx);
        (11, supports_le_coded_phy);
        (12, supports_le_extended_advertising);
        (13, supports_le_periodic_advertising);
        (14, supports_channel_selection_algorithm_2);
        (15, supports_le_power_class_1);
        (16, supports_min_used_channels_procedure);
        (17, supports_connection_cte_request);
        (18, supports_connection_cte_response);
        (19, supports_connectionless_cte_tx);
        (20, supports_connectionless_cte_rx);
        (21, supports_antenna_switching_during_cte_tx);
        (22, supports_antenna_switching_during_cte_rx);
        (23, supports_receiving_constant_tone_extensions);
        (24, supports_periodic_advertising_sync_transfer_sender);
        (25, supports_periodic_advertising_sync_transfer_recipient);
        (26, supports_sleep_clock_accuracy_updates);
        (27, supports_remote_public_key_validation);
        (28, supports_connected_isochronous_stream_central);
        (29, supports_connected_isochronous_stream_peripheral);
        (30, supports_isochronous_broadcaster);
        (31, supports_synchronized_receiver);
        (32, supports_connected_isochronous_stream);
        (33, supports_le_power_control_request);
        (35, supports_le_path_loss_monitoring);
        (36, supports_periodic_advertising_adi);
        (37, supports_connection_subrating);
        (38, supports_connection_subrating_host);
        (39, supports_channel_classification);
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Duration<const N: u32 = 1>([u8; 2]);

impl<const N: u32> Duration<N> {
    pub fn from_u16(val: u16) -> Self {
        Self(val.to_le_bytes())
    }

    pub fn from_micros(val: u32) -> Self {
        Self::from_u16((val / (625 * N)) as u16)
    }

    pub fn from_millis(val: u32) -> Self {
        Self::from_u16((unwrap!(val.checked_mul(8)) / (5 * N)) as u16)
    }

    pub fn from_secs(val: u32) -> Self {
        Self::from_millis(unwrap!(val.checked_mul(1000)))
    }

    pub fn to_raw(self) -> [u8; 2] {
        self.0
    }

    pub fn as_u16(&self) -> u16 {
        u16::from_le_bytes(self.0)
    }

    pub fn as_micros(&self) -> u32 {
        u32::from(self.as_u16()) * (625 * N)
    }

    pub fn as_millis(&self) -> u32 {
        (u32::from(self.as_u16()) * (5 * N)) / 8
    }

    pub fn as_secs(&self) -> u32 {
        self.as_millis() / 1000
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdvertisingType(u8);

impl AdvertisingType {
    pub const ADV_IND: AdvertisingType = AdvertisingType(0);
    pub const ADV_DIRECT_IND_HIGH: AdvertisingType = AdvertisingType(1);
    pub const ADV_SCAN_IND: AdvertisingType = AdvertisingType(2);
    pub const ADV_NONCONN_IND: AdvertisingType = AdvertisingType(3);
    pub const ADV_DIRECT_IND_LOW: AdvertisingType = AdvertisingType(4);

    pub const fn new(val: u8) -> Self {
        Self(val)
    }

    pub const fn to_raw(self) -> u8 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AddressType(u8);

impl AddressType {
    pub const PUBLIC: AddressType = AddressType(0);
    pub const RANDOM: AddressType = AddressType(1);
    pub const RESOLVABLE_PRIVATE_OR_PUBLIC: AddressType = AddressType(2);
    pub const RESOLVABLE_PRIVATE_OR_RANDOM: AddressType = AddressType(3);
    pub const ANONYMOUS_ADVERTISEMENTS: AddressType = AddressType(0xff);

    pub const fn new(val: u8) -> Self {
        Self(val)
    }

    pub const fn to_raw(self) -> u8 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdvertisingChannelMap(u8);

impl AdvertisingChannelMap {
    pub const ALL: AdvertisingChannelMap = AdvertisingChannelMap(0x07);
    pub const CHANNEL_37: AdvertisingChannelMap = AdvertisingChannelMap(0x01);
    pub const CHANNEL_38: AdvertisingChannelMap = AdvertisingChannelMap(0x02);
    pub const CHANNEL_39: AdvertisingChannelMap = AdvertisingChannelMap(0x04);

    pub const fn new(val: u8) -> Self {
        Self(val)
    }

    pub const fn to_raw(self) -> u8 {
        self.0
    }

    pub const fn is_channel_37_enabled(&self) -> bool {
        (self.0 & 1) != 0
    }

    pub fn enable_channel_37(&mut self, enable: bool) {
        self.0 = (self.0 & !1) | u8::from(enable)
    }

    pub const fn is_channel_38_enabled(&self) -> bool {
        (self.0 & 2) != 0
    }

    pub fn enable_channel_38(&mut self, enable: bool) {
        self.0 = (self.0 & !2) | (u8::from(enable) << 1)
    }

    pub const fn is_channel_39_enabled(&self) -> bool {
        (self.0 & 4) != 0
    }

    pub fn enable_channel_39(&mut self, enable: bool) {
        self.0 = (self.0 & !4) | (u8::from(enable) << 2)
    }
}

impl Default for AdvertisingChannelMap {
    fn default() -> Self {
        Self::ALL
    }
}

#[repr(transparent)]
#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdvertisingFilterPolicy(u8);

impl AdvertisingFilterPolicy {
    pub const UNFILTERED: AdvertisingFilterPolicy = AdvertisingFilterPolicy(0);
    pub const FILTER_SCAN: AdvertisingFilterPolicy = AdvertisingFilterPolicy(1);
    pub const FILTER_CONNECTION: AdvertisingFilterPolicy = AdvertisingFilterPolicy(2);
    pub const FILTER_CONNECTION_AND_SCAN: AdvertisingFilterPolicy = AdvertisingFilterPolicy(3);

    pub const fn new(val: u8) -> Self {
        Self(val)
    }

    pub const fn to_raw(self) -> u8 {
        self.0
    }
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LocalVersionInformation {
    hci_version: u8,
    hci_subversion: [u8; 2],
    lmp_version: u8,
    company_identifier: [u8; 2],
    lmp_subversion: [u8; 2],
}

impl LocalVersionInformation {
    pub fn new(
        hci_version: u8,
        hci_subversion: u16,
        lmp_version: u8,
        company_identifier: u16,
        lmp_subversion: u16,
    ) -> Self {
        Self {
            hci_version,
            hci_subversion: hci_subversion.to_le_bytes(),
            lmp_version,
            company_identifier: company_identifier.to_le_bytes(),
            lmp_subversion: lmp_subversion.to_le_bytes(),
        }
    }

    pub fn hci_version(&self) -> u8 {
        self.hci_version
    }

    pub fn hci_subversion(&self) -> u16 {
        u16::from_le_bytes(self.hci_subversion)
    }

    pub fn lmp_version(&self) -> u8 {
        self.lmp_version
    }

    pub fn company_identifier(&self) -> u16 {
        u16::from_le_bytes(self.company_identifier)
    }

    pub fn lmp_subversion(&self) -> u16 {
        u16::from_le_bytes(self.lmp_subversion)
    }
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LeReadBufferSize {
    le_acl_data_packet_length: [u8; 2],
    total_num_le_acl_data_packets: u8,
    iso_data_packet_length: [u8; 2],
    total_num_iso_data_packets: u8,
}

impl LeReadBufferSize {
    pub fn new(
        le_acl_data_packet_length: u16,
        total_num_le_acl_data_packets: u8,
        iso_data_packet_length: u16,
        total_num_iso_data_packets: u8,
    ) -> Self {
        Self {
            le_acl_data_packet_length: le_acl_data_packet_length.to_le_bytes(),
            total_num_le_acl_data_packets,
            iso_data_packet_length: iso_data_packet_length.to_le_bytes(),
            total_num_iso_data_packets,
        }
    }

    pub fn le_acl_data_packet_length(&self) -> u16 {
        u16::from_le_bytes(self.le_acl_data_packet_length)
    }

    pub fn total_num_le_acl_data_packets(&self) -> u8 {
        self.total_num_le_acl_data_packets
    }

    pub fn iso_data_packet_length(&self) -> u16 {
        u16::from_le_bytes(self.iso_data_packet_length)
    }

    pub fn total_num_iso_data_packets(&self) -> u8 {
        self.total_num_iso_data_packets
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LeScanType(u8);

impl LeScanType {
    pub const PASSIVE: LeScanType = LeScanType(0);
    pub const ACTIVE: LeScanType = LeScanType(1);

    pub const fn new(val: u8) -> Self {
        Self(val)
    }

    pub const fn to_raw(self) -> u8 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ScanningFilterPolicy(u8);

impl ScanningFilterPolicy {
    pub const BASIC_UNFILTERED: ScanningFilterPolicy = ScanningFilterPolicy(0);
    pub const BASIC_FILTERED: ScanningFilterPolicy = ScanningFilterPolicy(1);
    pub const EXTENDED_UNFILTERED: ScanningFilterPolicy = ScanningFilterPolicy(2);
    pub const EXTENDED_FILTERED: ScanningFilterPolicy = ScanningFilterPolicy(3);

    pub const fn new(val: u8) -> Self {
        Self(val)
    }

    pub const fn to_raw(self) -> u8 {
        self.0
    }
}

pub struct CoreSpecificationVersion(u8);

impl CoreSpecificationVersion {
    pub const VERSION_1_0B: CoreSpecificationVersion = CoreSpecificationVersion(0x00);
    pub const VERSION_1_1: CoreSpecificationVersion = CoreSpecificationVersion(0x01);
    pub const VERSION_1_2: CoreSpecificationVersion = CoreSpecificationVersion(0x02);
    pub const VERSION_2_0_EDR: CoreSpecificationVersion = CoreSpecificationVersion(0x03);
    pub const VERSION_2_1_EDR: CoreSpecificationVersion = CoreSpecificationVersion(0x04);
    pub const VERSION_3_0_HS: CoreSpecificationVersion = CoreSpecificationVersion(0x05);
    pub const VERSION_4_0: CoreSpecificationVersion = CoreSpecificationVersion(0x06);
    pub const VERSION_4_1: CoreSpecificationVersion = CoreSpecificationVersion(0x07);
    pub const VERSION_4_2: CoreSpecificationVersion = CoreSpecificationVersion(0x08);
    pub const VERSION_5_0: CoreSpecificationVersion = CoreSpecificationVersion(0x09);
    pub const VERSION_5_1: CoreSpecificationVersion = CoreSpecificationVersion(0x0A);
    pub const VERSION_5_2: CoreSpecificationVersion = CoreSpecificationVersion(0x0B);
    pub const VERSION_5_3: CoreSpecificationVersion = CoreSpecificationVersion(0x0C);

    pub fn to_raw(self) -> u8 {
        self.0
    }
}

mod error {
    use core::num::NonZeroU8;

    #[repr(transparent)]
    #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct Status(u8);

    impl Status {
        pub const SUCCESS: Status = Status(0);

        pub const fn new(n: u8) -> Self {
            Status(n)
        }

        pub const fn to_result(self) -> Result<(), Error> {
            if self.0 == Self::SUCCESS.0 {
                Ok(())
            } else {
                Err(Error(unsafe { NonZeroU8::new_unchecked(self.0) }))
            }
        }
    }

    #[cfg(feature = "defmt")]
    impl defmt::Format for Status {
        fn format(&self, fmt: defmt::Formatter) {
            defmt::Format::format(&self.to_result(), fmt)
        }
    }

    impl core::fmt::Debug for Status {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            core::fmt::Debug::fmt(&self.to_result(), f)
        }
    }

    impl From<u8> for Status {
        fn from(value: u8) -> Self {
            Status(value)
        }
    }

    impl From<Status> for u8 {
        fn from(value: Status) -> Self {
            value.0
        }
    }

    #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct Error(NonZeroU8);

    impl Error {
        const unsafe fn from_u8(err: u8) -> Error {
            Error(NonZeroU8::new_unchecked(err))
        }

        pub const fn to_status(self) -> Status {
            Status(self.0.get())
        }
    }

    impl core::fmt::Display for Error {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            core::fmt::Debug::fmt(self, f)
        }
    }

    impl From<Error> for u8 {
        fn from(value: Error) -> Self {
            value.0.get()
        }
    }

    macro_rules! errnos {
        (
            $(
                ($val:expr, $konst:ident, $desc:expr);
            )+
        ) => {
            impl Error {
            $(
                #[doc = $desc]
                pub const $konst: Error = unsafe { Error::from_u8($val) };
            )+
            }

            impl Status {
            $(
                #[doc = $desc]
                pub const $konst: Status = Error::$konst.to_status();
            )+
            }

            #[cfg(feature = "defmt")]
            impl defmt::Format for Error {
                fn format(&self, fmt: defmt::Formatter) {
                    match *self {
                        $(
                        Self::$konst => defmt::write!(fmt, $desc),
                        )+
                        _ => defmt::write!(fmt, "Unknown error: {}", self.0),
                    }
                }
            }

            impl core::fmt::Debug for Error {
                fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                    match *self {
                        $(
                        Self::$konst => core::write!(fmt, $desc),
                        )+
                        _ => core::write!(fmt, "Unknown errno: {}", self.0),
                    }
                }
            }
        }
    }

    errnos! {
        (0x01, UNKNOWN_COMMAND, "Unknown HCI Command");
        (0x02, UNKNOWN_CONNECTION_IDENTIFIER, "Unknown Connection Identifier");
        (0x03, HARDWARE_FAILURE, "Hardware Failure");
        (0x04, PAGE_TIMEOUT, "Page Timeout");
        (0x05, AUTHENTICATION_FAILURE, "Authentication Failure");
        (0x06, PIN_OR_KEY_MISSING, "PIN or Key Missing");
        (0x07, MEMORY_CAPACITY_EXCEEDED, "Memory Capacity Exceeded");
        (0x08, CONNECTION_TIMEOUT, "Connection Timeout");
        (0x09, CONNECTION_LIMIT_EXCEEDED, "Connection Limit Exceeded");
        (0x0A, SYNCHRONOUS_CONNECTION_LIMIT_EXCEEDED, "Synchronous Connection Limit To A Device Exceeded");
        (0x0B, CONNECTION_ALREADY_EXISTS, "Connection Already Exists");
        (0x0C, COMMAND_DISALLOWED, "Command Disallowed");
        (0x0D, CONNECTION_REJECTED_LIMITED_RESOURCES, "Connection Rejected due to Limited Resources");
        (0x0E, CONNECTION_REJECTED_SECURITY_REASONS, "Connection Rejected Due To Security Reasons");
        (0x0F, CONNECTION_REJECTED_UNACCEPTABLE_BD_ADDR, "Connection Rejected due to Unacceptable BD_ADDR");
        (0x10, CONNECTION_ACCEPT_TIMEOUT_EXCEEDED, "Connection Accept Timeout Exceeded");
        (0x11, UNSUPPORTED, "Unsupported Feature or Parameter Value");
        (0x12, INVALID_HCI_PARAMETERS, "Invalid HCI Command Parameters");
        (0x13, REMOTE_USER_TERMINATED_CONNECTION, "Remote User Terminated Connection");
        (0x14, REMOTE_DEVICE_TERMINATED_CONNECTION_LOW_RESOURCES, "Remote Device Terminated Connection due to Low Resources");
        (0x15, REMOTE_DEVICE_TERMINATED_CONNECTION_POWER_OFF, "Remote Device Terminated Connection due to Power Off");
        (0x16, CONNECTION_TERMINATED_BY_LOCAL_HOST, "Connection Terminated By Local Host");
        (0x17, REPEATED_ATTEMPTS, "Repeated Attempts");
        (0x18, PAIRING_NOT_ALLOWED, "Pairing Not Allowed");
        (0x19, UNKNOWN_LMP_PDU, "Unknown LMP PDU");
        (0x1A, UNSUPPORTED_REMOTE_FEATURE, "Unsupported Remote Feature");
        (0x1B, SCO_OFFSET_REJECTED, "SCO Offset Rejected");
        (0x1C, SCO_INTERVAL_REJECTED, "SCO Interval Rejected");
        (0x1D, SCO_AIR_MODE_REJECTED, "SCO Air Mode Rejected");
        (0x1E, INVALID_LMP_LL_PARAMETERS, "Invalid LMP Parameters / Invalid LL Parameters");
        (0x1F, UNSPECIFIED, "Unspecified Error");
        (0x20, UNSUPPORTED_LMP_LL_PARAMETER_VALUE, "Unsupported LMP Parameter Value / Unsupported LL Parameter Value");
        (0x21, ROLE_CHANGE_NOT_ALLOWED, "Role Change Not Allowed");
        (0x22, LMP_LL_RESPONSE_TIMEOUT, "LMP Response Timeout / LL Response Timeout");
        (0x23, LMP_LL_COLLISION, "LMP Error Transaction Collision / LL Procedure Collision");
        (0x24, LMP_PDU_NOT_ALLOWED, "LMP PDU Not Allowed");
        (0x25, ENCRYPTION_MODE_NOT_ACCEPTABLE, "Encryption Mode Not Acceptable");
        (0x26, LINK_KEY_CANNOT_BE_CHANGED, "Link Key cannot be Changed");
        (0x27, REQUESTED_QOS_NOT_SUPPORTED, "Requested QoS Not Supported");
        (0x28, INSTANT_PASSED, "Instant Passed");
        (0x29, PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED, "Pairing With Unit Key Not Supported");
        (0x2A, DIFFERENT_TRANSACTION_COLLISION, "Different Transaction Collision");
        (0x2C, QOS_UNACCEPTABLE_PARAMETER, "QoS Unacceptable Parameter");
        (0x2D, QOS_REJECTED, "QoS Rejected");
        (0x2E, CHANNEL_CLASSIFICATION_NOT_SUPPORTED, "Channel Classification Not Supported");
        (0x2F, INSUFFICIENT_SECURITY, "Insufficient Security");
        (0x30, PARAMETER_OUT_OF_RANGE, "Parameter Out Of Mandatory Range");
        (0x32, ROLE_SWITCH_PENDING, "Role Switch Pending");
        (0x34, RESERVED_SLOT_VIOLATION, "Reserved Slot Violation");
        (0x35, ROLE_SWITCH_FAILED, "Role Switch Failed");
        (0x36, EXTENDED_INQUIRY_RESPONSE_TOO_LARGE, "Extended Inquiry Response Too Large");
        (0x37, SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST, "Secure Simple Pairing Not Supported By Host");
        (0x38, HOST_BUSY_PAIRING, "Host Busy - Pairing");
        (0x39, CONNECTION_REJECTED_NO_SUITABLE_CHANNEL_FOUND, "Connection Rejected due to No Suitable Channel Found");
        (0x3A, CONTROLLER_BUSY, "Controller Busy");
        (0x3B, UNACCEPTABLE_CONNECTION_PARAMETERS, "Unacceptable Connection Parameters");
        (0x3C, ADVERTISING_TIMEOUT, "Advertising Timeout");
        (0x3D, CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE, "Connection Terminated due to MIC Failure");
        (0x3E, CONNECTION_FAILED_SYNCHRONIZATION_TIMEOUT, "Connection Failed to be Established / Synchronization Timeout");
        (0x40, COARSE_CLOCK_ADJUSTMENT_REJECTED, "Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock Dragging");
        (0x41, TYPE0_SUBMAP_NOT_DEFINED, "Type0 Submap Not Defined");
        (0x42, UNKNOWN_ADVERTISING_IDENTIFIER, "Unknown Advertising Identifier");
        (0x43, LIMIT_REACHED, "Limit Reached");
        (0x44, OPERATION_CANCELLED_BY_HOST, "Operation Cancelled by Host");
        (0x45, PACKET_TOO_LONG, "Packet Too Long");
    }
}

pub use error::*;
