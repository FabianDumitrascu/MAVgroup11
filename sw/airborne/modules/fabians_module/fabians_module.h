/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/fabians_module/fabians_module.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

 #ifndef FABIANS_MODULE_H
 #define FABIANS_MODULE_H
 
 // settings
 extern float oa_color_count_frac;
 
 // functions
 extern void fabians_module_init(void);
 extern void fabians_module_periodic(void);
 
 #endif
 
 