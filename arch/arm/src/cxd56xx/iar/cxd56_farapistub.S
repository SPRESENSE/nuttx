/****************************************************************************
 * arch/arm/src/cxd56xx/iar/cxd56_farapistub.S
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

    EXTERN farapi_main

    SECTION `.text`:CODE
    THUMB

fw_power_mgr_stub1:
    push    {r0-r3}
    ldr     r1, =fw_power_mgr_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_power_mgr
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_power_mgr_stub2:

    /* fw_pm_gpspllclocksetsource */
    nop
    nop

    /* fw_pm_pmicgetchargingporttype */
    nop
    nop

    /* fw_pm_pmicsetchargingcurrentvalue */
    nop
    nop

    /* fw_pm_pmicstartcharging */
    nop
    nop

    /* fw_pm_pmicstopcharging */
    nop
    nop

    /* fw_pm_peripoweron */
    nop
    nop

    /* fw_pm_peripoweroff */
    nop
    nop

    /* fw__pm_periclockenable */
    nop
    nop

    /* fw__pm_periclockdisable */
    nop
    nop

    /* fw__pm_updatecpufreqinfo */
    nop
    nop

    /* fw_pm_pmicpoweron */
    nop
    nop

    /* fw_pm_pmicpoweroff */
    nop
    nop

    /* fw_pm_setwakeuptrigger */
    nop
    nop

    /* fw_pm_hostifclockenable */
    nop
    nop

    /* fw_pm_hostifclockdisable */
    nop
    nop

    PUBLIC 	fw_pm_startcpu
    THUMB
fw_pm_startcpu:
    mov     r12, pc
    b       fw_power_mgr_stub1

    PUBLIC 	fw_pm_stopcpu
    THUMB
fw_pm_stopcpu:
    mov     r12, pc
    b       fw_power_mgr_stub1

    PUBLIC 	fw_pm_sleepcpu
    THUMB
fw_pm_sleepcpu:
    mov     r12, pc
    b       fw_power_mgr_stub1

    PUBLIC 	fw_pm_wakeupcpu
    THUMB
fw_pm_wakeupcpu:
    mov     r12, pc
    b       fw_power_mgr_stub1

    PUBLIC 	fw_pm_coldsleep
    THUMB
fw_pm_coldsleep:
    mov     r12, pc
    b       fw_power_mgr_stub1

    PUBLIC 	fw_pm_deepsleep
    THUMB
fw_pm_deepsleep:
    mov     r12, pc
    b       fw_power_mgr_stub1

    PUBLIC 	fw_pm_reboot
    THUMB
fw_pm_reboot:
    mov     r12, pc
    b       fw_power_mgr_stub1

    PUBLIC 	fw_pm_loadimage
    THUMB
fw_pm_loadimage:
    mov     r12, pc
    b       fw_power_mgr_stub1

    /* fw_pm_loadimagetoaddress */
    nop
    nop

    /* fw_pm_ramcontrolbyaddress */
    nop
    nop

    /* fw_pm_changeclock */
    nop
    nop

    PUBLIC 	fw_pm_pmiccontrol
    THUMB
fw_pm_pmiccontrol:
    mov     r12, pc
    b       fw_power_mgr_stub1

    /* fw_pm_bootcpu */
    nop
    nop

    /* fw_pm_bootcpuwithwaitmode */
    nop
    nop

    /* fw_pm_getclock */
    nop
    nop

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM 4
    PUBLIC   _modulelist_power_mgr
_modulelist_power_mgr:
    EXTWEAK _module_power_mgr
    DC32    _module_power_mgr
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_flash_mgr_stub1:
    push    {r0-r3}
    ldr     r1, =fw_flash_mgr_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_flash_mgr
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_flash_mgr_stub2:

    /* fw_fm_getregistryvalue */
    nop
    nop

    /* fw_fm_setregistryvalue */
    nop
    nop

    /* fw_fm_setregistryvaluelist */
    nop
    nop

    /* fw_fm_deleteregistrykey */
    nop
    nop

    /* fw_fm_generateregistrykey */
    nop
    nop

    /* fw_fm_sync */
    nop
    nop

    /* fw_fm_fileopen */
    nop
    nop

    /* fw_fm_fileclose */
    nop
    nop

    /* fw_fm_fileread */
    nop
    nop

    /* fw_fm_filewrite */
    nop
    nop

    /* fw_fm_fileseek */
    nop
    nop

    /* fw_fm_fileflush */
    nop
    nop

    /* fw_fm_filedelete */
    nop
    nop

    PUBLIC 	fw_fm_rawwrite
    THUMB
fw_fm_rawwrite:
    mov     r12, pc
    b       fw_flash_mgr_stub1

    PUBLIC 	fw_fm_rawread
    THUMB
fw_fm_rawread:
    mov     r12, pc
    b       fw_flash_mgr_stub1

    PUBLIC 	fw_fm_rawerasesector
    THUMB
fw_fm_rawerasesector:
    mov     r12, pc
    b       fw_flash_mgr_stub1

    /* fw_fm_filerename */
    nop
    nop

    /* fw_fm_filegetattr */
    nop
    nop

    /* fw_fm_filesetattr */
    nop
    nop

    /* fw_fm_fileclearattr */
    nop
    nop

    PUBLIC 	fw_fm_rawverifywrite
    THUMB
fw_fm_rawverifywrite:
    mov     r12, pc
    b       fw_flash_mgr_stub1

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM 4
    PUBLIC   _modulelist_flash_mgr
_modulelist_flash_mgr:
    EXTWEAK _module_flash_mgr
    DC32    _module_flash_mgr
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_rtc_mgr_stub1:
    push    {r0-r3}
    ldr     r1, =fw_rtc_mgr_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_rtc_mgr
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_rtc_mgr_stub2:

    /* fw_sys_rtcmgrsettime */
    nop
    nop

    /* fw_sys_rtcmgrgettime */
    nop
    nop

    /* fw_sys_rtcmgrconvtime */
    nop
    nop

    /* fw_sys_rtcmgrconvcalendartime */
    nop
    nop

    /* fw_sys_rtcmgrsetcalendartime */
    nop
    nop

    /* fw_sys_rtcmgrgetcalendartime */
    nop
    nop

    /* fw_sys_rtcmgradjusttime */
    nop
    nop

    /* fw_sys_rtcmgrrequestalarm */
    nop
    nop

    /* fw_sys_rtcmgrcancelalarm */
    nop
    nop

    SECTION .modulelist:CONST:NOROOT(4)
    PUBLIC   _modulelist_rtc_mgr
_modulelist_rtc_mgr:
    EXTWEAK _module_rtc_mgr
    DC32    _module_rtc_mgr
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_gnss_pwr_stub1:
    push    {r0-r3}
    ldr     r1, =fw_gnss_pwr_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_gnss_pwr
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_gnss_pwr_stub2:

    /* fw_gnsp_changeclockboot */
    nop
    nop

    /* fw_gnsp_changeclocknormal */
    nop
    nop

    /* fw_gnsp_setattribute */
    nop
    nop

    /* fw_gnsp_getpowerstatus */
    nop
    nop

    /* fw_gnsp_pongnss */
    nop
    nop

    /* fw_gnsp_poffgnss */
    nop
    nop

    /* fw_gnsp_pofflna */
    nop
    nop

    /* fw_gnsp_sleep */
    nop
    nop

    /* fw_gnsp_wakeup */
    nop
    nop

    /* fw_gnsp_setwakeuptime */
    nop
    nop

    /* fw_gnsp_getattribute */
    nop
    nop

    /* fw_gnsp_clockon */
    nop
    nop

    /* fw_gnsp_clockoff */
    nop
    nop

    /* fw_gnsp_changemode */
    nop
    nop

    /* fw_gnsp_enableio */
    nop
    nop

    /* fw_gnsp_controlgpioout */
    nop
    nop

    /* fw_gnsp_debug */
    nop
    nop

    /* fw_gnsp_changeclockrtc */
    nop
    nop

    /* fw_gnsp_changeclocksfcclkup */
    nop
    nop

    /* fw_gnsp_changeclocksfcclkdwn */
    nop
    nop

    /* fw_gnsp_changeclockgpscpuclkup */
    nop
    nop

    /* fw_gnsp_changeclockgpscpuclkdwn */
    nop
    nop

    /* fw_gnsp_getbootcause */
    nop
    nop

    /* fw_gnsp_getwakeuptrigger */
    nop
    nop

    /* fw_gnsp_selfreboot */
    nop
    nop

    /* fw_gnsp_start_keepppsout */
    nop
    nop

    /* fw_gnsp_stop_keepppsout */
    nop
    nop

    /* fw_gnsp_start_keepitpout */
    nop
    nop

    /* fw_gnsp_stop_keepitpout */
    nop
    nop

    /* fw_gnsp_setsleepstate */
    nop
    nop

    /* fw_gnsp_freesysmemory */
    nop
    nop

    /* fw_gnsp_readalmanacfile */
    nop
    nop

    SECTION .modulelist:CONST:NOROOT(4)
    PUBLIC   _modulelist_gnss_pwr
_modulelist_gnss_pwr:
    EXTWEAK _module_gnss_pwr
    DC32    _module_gnss_pwr
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_aca_stub1:
    push    {r0-r3}
    ldr     r1, =fw_aca_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_aca
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_aca_stub2:

    PUBLIC 	fw_as_acacontrol
    THUMB
fw_as_acacontrol:
    mov     r12, pc
    b       fw_aca_stub1

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM	2
    PUBLIC   _modulelist_aca
_modulelist_aca:
    EXTWEAK _module_aca
    DC32    _module_aca
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_pinconfig_stub1:
    push    {r0-r3}
    ldr     r1, =fw_pinconfig_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_pinconfig
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_pinconfig_stub2:

    PUBLIC 	fw_pd_pinconfigs
    THUMB
fw_pd_pinconfigs:
    mov     r12, pc
    b       fw_pinconfig_stub1

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM	2
    PUBLIC   _modulelist_pinconfig
_modulelist_pinconfig:
    EXTWEAK _module_pinconfig
    DC32    _module_pinconfig
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_uart_stub1:
    push    {r0-r3}
    ldr     r1, =fw_uart_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_uart
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_uart_stub2:

    PUBLIC 	fw_pd_uartinit
    THUMB
fw_pd_uartinit:
    mov     r12, pc
    b       fw_uart_stub1

    PUBLIC 	fw_pd_uartuninit
    THUMB
fw_pd_uartuninit:
    mov     r12, pc
    b       fw_uart_stub1

    PUBLIC 	fw_pd_uartconfiguration
    THUMB
fw_pd_uartconfiguration:
    mov     r12, pc
    b       fw_uart_stub1

    PUBLIC 	fw_pd_uartenable
    THUMB
fw_pd_uartenable:
    mov     r12, pc
    b       fw_uart_stub1

    PUBLIC 	fw_pd_uartdisable
    THUMB
fw_pd_uartdisable:
    mov     r12, pc
    b       fw_uart_stub1

    PUBLIC 	fw_pd_uartreceive
    THUMB
fw_pd_uartreceive:
    mov     r12, pc
    b       fw_uart_stub1

    PUBLIC 	fw_pd_uartsend
    THUMB
fw_pd_uartsend:
    mov     r12, pc
    b       fw_uart_stub1

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM	2
    PUBLIC   _modulelist_uart
_modulelist_uart:
    EXTWEAK _module_uart
    DC32    _module_uart
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_update_mgr_stub1:
    push    {r0-r3}
    ldr     r1, =fw_update_mgr_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_update_mgr
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_update_mgr_stub2:

    PUBLIC 	fw_um_init
    THUMB
fw_um_init:
    mov     r12, pc
    b       fw_update_mgr_stub1

    PUBLIC 	fw_um_open
    THUMB
fw_um_open:
    mov     r12, pc
    b       fw_update_mgr_stub1

    PUBLIC 	fw_um_commit
    THUMB
fw_um_commit:
    mov     r12, pc
    b       fw_update_mgr_stub1

    PUBLIC 	fw_um_close
    THUMB
fw_um_close:
    mov     r12, pc
    b       fw_update_mgr_stub1

    PUBLIC 	fw_um_checkpoint
    THUMB
fw_um_checkpoint:
    mov     r12, pc
    b       fw_update_mgr_stub1

    PUBLIC 	fw_um_doupdatesequence
    THUMB
fw_um_doupdatesequence:
    mov     r12, pc
    b       fw_update_mgr_stub1

    PUBLIC 	fw_um_abort
    THUMB
fw_um_abort:
    mov     r12, pc
    b       fw_update_mgr_stub1

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM	2
    PUBLIC   _modulelist_update_mgr
_modulelist_update_mgr:
    EXTWEAK _module_update_mgr
    DC32    _module_update_mgr
    DC32    0
    DC32    0
    DC32    0

    SECTION `.text`:CODE
    THUMB

fw_gnss_stub1:
    push    {r0-r3}
    ldr     r1, =fw_gnss_stub2
    sub     r1, r1, #1
    mov     r0, r12
    subs    r0, r0, r1
    mov     r1, sp
    ldr     r2, =_modulelist_gnss
    push    {r4, lr}
    bl      farapi_main
    pop     {r0, r1}
    mov     r12, r1
    pop     {r0-r3}
    bx      r12

fw_gnss_stub2:

    PUBLIC 	fw_gd_start
    THUMB
fw_gd_start:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_stop
    THUMB
fw_gd_stop:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_selectsatellitesystem
    THUMB
fw_gd_selectsatellitesystem:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_getsatellitesystem
    THUMB
fw_gd_getsatellitesystem:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setreceiverpositionellipsoidal
    THUMB
fw_gd_setreceiverpositionellipsoidal:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setreceiverpositionorthogonal
    THUMB
fw_gd_setreceiverpositionorthogonal:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setoperationmode
    THUMB
fw_gd_setoperationmode:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_getoperationmode
    THUMB
fw_gd_getoperationmode:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_settcxooffset
    THUMB
fw_gd_settcxooffset:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_gettcxooffset
    THUMB
fw_gd_gettcxooffset:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_settime
    THUMB
fw_gd_settime:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_getalmanac
    THUMB
fw_gd_getalmanac:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setalmanac
    THUMB
fw_gd_setalmanac:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_getephemeris
    THUMB
fw_gd_getephemeris:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setephemeris
    THUMB
fw_gd_setephemeris:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_getvarephemeris
    THUMB
fw_gd_getvarephemeris:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setvarephemeris
    THUMB
fw_gd_setvarephemeris:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_startgpstest
    THUMB
fw_gd_startgpstest:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_stopgpstest
    THUMB
fw_gd_stopgpstest:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_getgpstestresult
    THUMB
fw_gd_getgpstestresult:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_savebackupdata
    THUMB
fw_gd_savebackupdata:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_erasebackup
    THUMB
fw_gd_erasebackup:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_cepsetassistdata
    THUMB
fw_gd_cepsetassistdata:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_ceperaseassistdata
    THUMB
fw_gd_ceperaseassistdata:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_cepcheckassistdata
    THUMB
fw_gd_cepcheckassistdata:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_cepgetagedata
    THUMB
fw_gd_cepgetagedata:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_cepinitassistdata
    THUMB
fw_gd_cepinitassistdata:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setacquist
    THUMB
fw_gd_setacquist:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setframetime
    THUMB
fw_gd_setframetime:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_settaugps
    THUMB
fw_gd_settaugps:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_settimegps
    THUMB
fw_gd_settimegps:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_clearreceiverinfo
    THUMB
fw_gd_clearreceiverinfo:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_settowassist
    THUMB
fw_gd_settowassist:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setutcmodel
    THUMB
fw_gd_setutcmodel:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_spectrumcontrol
    THUMB
fw_gd_spectrumcontrol:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_readbuffer
    THUMB
fw_gd_readbuffer:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_writebuffer
    THUMB
fw_gd_writebuffer:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setnotifymask
    THUMB
fw_gd_setnotifymask:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geoaddregion
    THUMB
fw_gd_geoaddregion:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geomodifyregion
    THUMB
fw_gd_geomodifyregion:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geodeleteregione
    THUMB
fw_gd_geodeleteregione:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geodeleteallregion
    THUMB
fw_gd_geodeleteallregion:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geogetregiondata
    THUMB
fw_gd_geogetregiondata:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geogetusedregionid
    THUMB
fw_gd_geogetusedregionid:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geosetopmode
    THUMB
fw_gd_geosetopmode:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_geosetallrgionnotifyrequest
    THUMB
fw_gd_geosetallrgionnotifyrequest:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_registergeofence
    THUMB
fw_gd_registergeofence:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_releasegeofence
    THUMB
fw_gd_releasegeofence:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_registerpvtlog
    THUMB
fw_gd_registerpvtlog:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_releasepvtlog
    THUMB
fw_gd_releasepvtlog:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_pvtlogdeletelog
    THUMB
fw_gd_pvtlogdeletelog:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_pvtloggetlogstatus
    THUMB
fw_gd_pvtloggetlogstatus:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtkstart
    THUMB
fw_gd_rtkstart:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtkstop
    THUMB
fw_gd_rtkstop:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtksetoutputinterval
    THUMB
fw_gd_rtksetoutputinterval:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtkgetoutputinterval
    THUMB
fw_gd_rtkgetoutputinterval:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtksetgnss
    THUMB
fw_gd_rtksetgnss:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtkgetgnss
    THUMB
fw_gd_rtkgetgnss:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtksetephnotify
    THUMB
fw_gd_rtksetephnotify:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_rtkgetephnotify
    THUMB
fw_gd_rtkgetephnotify:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_setusecase
    THUMB
fw_gd_setusecase:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_getusecase
    THUMB
fw_gd_getusecase:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_set1ppsoutput
    THUMB
fw_gd_set1ppsoutput:
    mov     r12, pc
    b       fw_gnss_stub1

    PUBLIC 	fw_gd_get1ppsoutput
    THUMB
fw_gd_get1ppsoutput:
    mov     r12, pc
    b       fw_gnss_stub1

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM	2
    PUBLIC   _modulelist_gnss
_modulelist_gnss:
    EXTWEAK _module_gnss
    DC32    _module_gnss
    DC32    1
    DC32    0
    DC32    0

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM	2
    PUBLIC   _modulelist_gnss_geofence
_modulelist_gnss_geofence:
    EXTWEAK _module_gnss_geofence
    DC32    _module_gnss_geofence
    DC32    1
    DC32    0
    DC32    0

    SECTION .modulelist:CONST:NOROOT(4)
    ALIGNROM	2
    PUBLIC   _modulelist_gnss_pvtlog
_modulelist_gnss_pvtlog:
    EXTWEAK _module_gnss_pvtlog
    DC32    _module_gnss_pvtlog
    DC32    1
    DC32    0
    DC32    0

    END
