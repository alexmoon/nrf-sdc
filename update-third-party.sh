#!/usr/bin/env bash
set -euo pipefail

# Dependency versions
CMSIS5_REPO="https://github.com/ARM-software/CMSIS_5.git"
CMSIS5_REF="master"

NRFX_REPO="https://github.com/NordicSemiconductor/nrfx.git"
NRFX_REF="v4.5.0"

NRFXLIB_REPO="https://github.com/nrfconnect/sdk-nrfxlib.git"
NRFXLIB_REF="v3.4.0"

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
rm -rf "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf51"
rm -rf "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf71"
rm -rf "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf91"
rm -rf "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf92"

# ── nrfxlib: mpsl + softdevice_controller headers and libs ──
sparse_clone "$NRFXLIB_REPO" "$NRFXLIB_REF" "$THIRD_PARTY/nordic/nrfxlib" \
    mpsl/include \
    mpsl/lib/nrf52 mpsl/lib/nrf53 mpsl/lib/nrf54l mpsl/lib/nrf54l_ns mpsl/lib/nrf54lm mpsl/lib/nrf54h \
    mpsl/fem/include mpsl/fem/common \
    mpsl/fem/simple_gpio mpsl/fem/nrf21540_gpio mpsl/fem/nrf21540_gpio_spi \
    softdevice_controller/include \
    softdevice_controller/lib/nrf52 softdevice_controller/lib/nrf53 \
    softdevice_controller/lib/nrf54l softdevice_controller/lib/nrf54l_ns softdevice_controller/lib/nrf54h \
    softdevice_controller/lib/nrf54lm softdevice_controller/lib/nrf54lv softdevice_controller/lib/nrf54ls
