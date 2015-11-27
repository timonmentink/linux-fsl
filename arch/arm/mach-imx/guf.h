/*
 * Copyright (C) 2013 Garz & Fricke GmbH
 */

#ifndef __GUF_H_
#include "linux/guf_xml_config.h"
#include <linux/gpio.h>

extern int guf_init_gpio_group(struct gpio *gpios, int num, bool bExport, bool bFixedDir);
extern struct guf_xml_data * guf_parse_xml_parameters(void);

int guf_xml_get_mac_address( const struct guf_xml_data_network * const network_config, u8 mac_addr[ETH_ALEN]);
int guf_setup_network_from_xml(const struct guf_xml_data_network * config );
#endif
