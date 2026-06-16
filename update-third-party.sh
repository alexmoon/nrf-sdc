#!/usr/bin/env bash
set -euo pipefail

# Dependency versions
CMSIS5_REPO="https://github.com/ARM-software/CMSIS_5.git"
CMSIS5_REF="master"

NRFX_REPO="https://github.com/NordicSemiconductor/nrfx.git"
NRFX_REF="v4.3.0"

NRFXLIB_REPO="https://github.com/nrfconnect/sdk-nrfxlib.git"
NRFXLIB_REF="v3.3.0"

THIRD_PARTY="nrf-mpsl-sys/third_party"

sparse_clone() {
    local repo="$1"
    local ref="$2"
    local dest="$3"
    shift 3
    local dirs=("$@")

    echo "Fetching $repo @ $ref -> $dest"

    rm -rf "$dest"
    mkdir -p "$dest"

    git -C "$dest" init -q
    git -C "$dest" remote add origin "$repo"
    git -C "$dest" sparse-checkout init --cone
    git -C "$dest" sparse-checkout set "${dirs[@]}"
    git -C "$dest" fetch --filter=blob:none --depth 1 origin "$ref"
    git -C "$dest" checkout FETCH_HEAD
    rm -rf "$dest/.git"

    echo "  Done."
}

# Delete all files in a directory except those named in the remaining arguments
keep_only() {
    local dir="$1"
    shift
    local keep=("$@")

    local find_args=()
    for f in "${keep[@]}"; do
        find_args+=( ! -name "$f" )
    done
    find "$dir" -maxdepth 1 -type f "${find_args[@]}" -delete
}

# ── CMSIS_5: only the Core headers + license ──
sparse_clone "$CMSIS5_REPO" "$CMSIS5_REF" "$THIRD_PARTY/arm/CMSIS_5" \
    CMSIS/Core/Include
# Remove top-level files we don't need (PDFs, build scripts, etc.)
keep_only "$THIRD_PARTY/arm/CMSIS_5" LICENSE.txt

# ── nrfx: specific headers from bsp (mdk/soc), drivers, hal, haly, templates ──
# Note: nrfx 4.x moved the MDK under bsp/stable/mdk and added the BSP layer
# (bsp/stable/nrfx_bsp.h, bsp/stable/soc/*) pulled in by drivers/nrfx_common.h.
sparse_clone "$NRFX_REPO" "$NRFX_REF" "$THIRD_PARTY/nordic/nrfx" \
    bsp drivers hal haly templates
# Remove top-level files except LICENSE and nrfx.h
keep_only "$THIRD_PARTY/nordic/nrfx" LICENSE nrfx.h
# drivers: only need the top-level .h files, not include/ or src/ subdirs
rm -rf "$THIRD_PARTY/nordic/nrfx/drivers/include" \
       "$THIRD_PARTY/nordic/nrfx/drivers/src"
# hal: only need nrf_common.h and nrf_clock.h
keep_only "$THIRD_PARTY/nordic/nrfx/hal" nrf_common.h nrf_clock.h
# haly: only need nrfy_common.h
keep_only "$THIRD_PARTY/nordic/nrfx/haly" nrfy_common.h
# templates: only need nrfx_glue.h
keep_only "$THIRD_PARTY/nordic/nrfx/templates" nrfx_glue.h
# bsp/stable: keep the BSP entry headers; soc layer kept whole (~0.5MB);
# the per-chip nrfx_config_* templates are not needed (local include/nrfx_config.h overrides).
keep_only "$THIRD_PARTY/nordic/nrfx/bsp/stable" nrfx_bsp.h nrfx_ext.h
rm -rf "$THIRD_PARTY/nordic/nrfx/bsp/stable/templates"
# bsp/stable/mdk: strip down to only the headers we need (full dir is ~270MB).
# nrf_erratas.h pulls in every chip's *_erratas.h, so all of those are required.
keep_only "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk" \
    nrf.h nrf_peripherals.h nrf_mem.h nrf_erratas.h \
    compiler_abstraction.h system_nrf.h \
    nrf51_erratas.h nrf52_erratas.h nrf53_erratas.h \
    nrf54l_erratas.h nrf54h_erratas.h nrf91_erratas.h \
    nrf52840.h nrf52840_bitfields.h nrf52840_peripherals.h \
    nrf52840_xxaa_memory.h nrf52840_name_change.h \
    nrf51_to_nrf52840.h nrf52_to_nrf52840.h system_nrf52840.h \
    nrf5340_network.h nrf5340_network_bitfields.h nrf5340_network_peripherals.h \
    nrf5340_network_name_change.h nrf5340_xxaa_network_memory.h system_nrf5340_network.h \
    nrf54l15.h nrf54l15_types.h nrf54l15_global.h \
    nrf54l15_application.h nrf54l15_application_peripherals.h \
    nrf54l15_xxaa_application_memory.h nrf54l15_flpr.h \
    nrf54l15_peripherals.h nrf54l15_interim.h nrf54l15_name_change.h \
    nrf54lm20a.h nrf54lm20a_types.h nrf54lm20a_global.h \
    nrf54lm20a_application.h nrf54lm20a_application_peripherals.h \
    nrf54lm20a_xxaa_application_memory.h nrf54lm20a_flpr.h \
    nrf54lm20a_peripherals.h nrf54lm20a_interim.h nrf54lm20a_name_change.h \
    nrf54ls05b.h nrf54ls05b_types.h nrf54ls05b_global.h \
    nrf54ls05b_application.h nrf54ls05b_application_peripherals.h \
    nrf54ls05b_xxaa_application_memory.h \
    nrf54ls05b_peripherals.h nrf54ls05b_interim.h nrf54ls05b_name_change.h \
    nrf54lv10a.h nrf54lv10a_types.h nrf54lv10a_global.h \
    nrf54lv10a_application.h nrf54lv10a_application_peripherals.h \
    nrf54lv10a_xxaa_application_memory.h nrf54lv10a_flpr.h \
    nrf54lv10a_peripherals.h nrf54lv10a_interim.h nrf54lv10a_name_change.h \
    nrf54h20.h nrf54h20_types.h nrf54h20_global.h \
    nrf54h20_application.h nrf54h20_application_peripherals.h \
    nrf54h20_flpr.h nrf54h20_ppr.h \
    nrf54h20_radiocore.h nrf54h20_radiocore_peripherals.h \
    nrf54h20_xxaa_radiocore_memory.h nrf54h20_peripherals.h \
    nrf54h20_secure.h nrf54h20_sysctrl.h \
    nrf54h20_interim.h nrf54h20_name_change.h haltium_interim.h

# ── nrfxlib: mpsl + softdevice_controller headers and libs ──
sparse_clone "$NRFXLIB_REPO" "$NRFXLIB_REF" "$THIRD_PARTY/nordic/nrfxlib" \
    mpsl/include \
    mpsl/lib/nrf52 mpsl/lib/nrf53 mpsl/lib/nrf54l mpsl/lib/nrf54l_ns mpsl/lib/nrf54h \
    mpsl/fem/include mpsl/fem/common \
    mpsl/fem/simple_gpio mpsl/fem/nrf21540_gpio mpsl/fem/nrf21540_gpio_spi \
    softdevice_controller/include \
    softdevice_controller/lib/nrf52 softdevice_controller/lib/nrf53 \
    softdevice_controller/lib/nrf54l softdevice_controller/lib/nrf54l_ns softdevice_controller/lib/nrf54h \
    softdevice_controller/lib/nrf54lm softdevice_controller/lib/nrf54lv softdevice_controller/lib/nrf54ls 

# Verify key files exist
echo ""
echo "Verifying fetched files..."
MISSING=0
for f in \
    "$THIRD_PARTY/arm/CMSIS_5/LICENSE.txt" \
    "$THIRD_PARTY/arm/CMSIS_5/CMSIS/Core/Include/core_cm4.h" \
    "$THIRD_PARTY/arm/CMSIS_5/CMSIS/Core/Include/core_cm33.h" \
    "$THIRD_PARTY/nordic/nrfx/nrfx.h" \
    "$THIRD_PARTY/nordic/nrfx/bsp/stable/nrfx_bsp.h" \
    "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf.h" \
    "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf52840.h" \
    "$THIRD_PARTY/nordic/nrfx/drivers/nrfx_common.h" \
    "$THIRD_PARTY/nordic/nrfx/hal/nrf_common.h" \
    "$THIRD_PARTY/nordic/nrfx/haly/nrfy_common.h" \
    "$THIRD_PARTY/nordic/nrfx/templates/nrfx_glue.h" \
    "$THIRD_PARTY/nordic/nrfxlib/LICENSE" \
    "$THIRD_PARTY/nordic/nrfxlib/mpsl/include/mpsl.h" \
    "$THIRD_PARTY/nordic/nrfxlib/softdevice_controller/include/sdc.h" \
; do
    if [ ! -f "$f" ]; then
        echo "  MISSING: $f"
        MISSING=1
    fi
done

if [ "$MISSING" -eq 0 ]; then
    echo "All key files present."
else
    echo "ERROR: Some expected files are missing!"
    exit 1
fi
