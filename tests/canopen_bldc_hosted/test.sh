#!/usr/bin/env bash
(trap 'kill 0' SIGINT; build/canopen_bldc_hosted/scons-release/canopen_bldc_hosted.elf 4 & build/canopen_bldc_hosted/scons-release/canopen_bldc_hosted.elf 6 & build/canopen_bldc_hosted/scons-release/canopen_bldc_hosted.elf 8 & build/canopen_bldc_hosted/scons-release/canopen_bldc_hosted.elf 10 & build/canopen_bldc_hosted/scons-release/canopen_bldc_hosted.elf 12 & build/canopen_bldc_hosted/scons-release/canopen_bldc_hosted.elf 16 & build/canopen_bldc_hosted/scons-release/canopen_bldc_hosted.elf 18)
