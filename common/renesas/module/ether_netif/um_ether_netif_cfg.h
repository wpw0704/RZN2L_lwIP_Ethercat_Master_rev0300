/*
* Copyright (c) 2020 - 2025 Renesas Electronics Corporation and/or its affiliates
*
* SPDX-License-Identifier: BSD-3-Clause
*/
/**********************************************************************************************************************
 * File Name    :um_ether_netif_cfg.h
 * Version      :1
 *********************************************************************************************************************/
/**********************************************************************************************************************
 * History :
 *
 *
 *********************************************************************************************************************/
#ifndef UM_ETHER_NETIF_CFG_H_
#define UM_ETHER_NETIF_CFG_H_
/** Implements NULL checking of parameters between the Ethernet API and the driver.
 *  "1": Enabled "0": Disabled*/
#define UM_ETHER_NETIF_CFG_PARAM_CHECKING_ENABLE        (1)
/** Specify the task name of the Ethernet receive task.*/
#define UM_ETHER_NETIF_CFG_READER_TASK_NAME             ("Ethernet Reader Task")
/** Specify the priority of the Ethernet receive task */
#define UM_ETHER_NETIF_CFG_READER_TASK_PRIORITY         (8)
/** Specify the stack size for the Ethernet receive task. */
#define UM_ETHER_NETIF_CFG_READER_TASK_STACK_BYTE_SIZE  (1024)
/** Specify the task name for the Ethernet status monitoring task.*/
#define UM_ETHER_NETIF_CFG_MONITOR_TASK_NAME            ("Ethernet Monitor Task")
/** Specify the priority of the Ethernet status monitoring task*/
#define UM_ETHER_NETIF_CFG_MONITOR_TASK_PRIORITY        (3)
/** Specifies the stack size for the Ethernet status monitoring task.*/
#define UM_ETHER_NETIF_CFG_MONITOR_TASK_STACK_BTYE_SIZE (512)
/** Maximum Transmission Unit */
#define UM_ETHER_NETIF_CFG_MTU_BYTES                    (1500)

#endif /** UM_ETHER_NETIF_CFG_H_ */
