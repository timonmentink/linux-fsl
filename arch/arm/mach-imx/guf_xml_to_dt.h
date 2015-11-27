/*
 * Copyright (C) 2014 Garz & Fricke GmbH
 */
#ifndef __GUF_XML_TO_DT__
#define __GUF_XML_TO_DT__

#ifdef CONFIG_FB
int set_video_mode_from_xml(struct guf_xml_data * xml_config);
int set_backlight_settings_from_xml(guf_xml_data_backlight_t * xml_bl_setting);
int configure_touch_from_xml( const guf_xml_data_touch_t * xml_touch);
int logo_license_from_dt_to_xml(guf_xml_data_logo_license_t * license, guf_xml_data_rotation_t * rotation);
#endif
int set_mac_addr_from_xml(const struct guf_xml_data_network * const network_config);
int configure_keypad(char * keypad_from_xml);
int integrate_devtree_from_xml(const struct guf_xml_devtree * devtree);
int update_devicetree_to_board_revision( int major, int minor);


#endif

