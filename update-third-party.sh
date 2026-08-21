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

# Delete top-level files in a directory except those named in the remaining arguments.
keep_only() {
    local dir="$1"
    shift
    local keep=("$@")

    local find_args=()
    local file
    for file in "${keep[@]}"; do
        find_args+=( ! -name "$file" )
    done
    find "$dir" -maxdepth 1 -type f "${find_args[@]}" -delete
}

# Delete every file below a directory except the explicitly listed relative paths,
# then remove directories left empty by the pruning.
keep_only_paths() {
    local dir="$1"
    shift
    local keep=("$@")

    local file relative candidate found
    for candidate in "${keep[@]}"; do
        if [[ ! -f "$dir/$candidate" ]]; then
            echo "Required third-party file is missing: $dir/$candidate" >&2
            return 1
        fi
    done

    while IFS= read -r -d '' file; do
        relative="${file#"$dir"/}"
        found=0
        for candidate in "${keep[@]}"; do
            if [[ "$relative" == "$candidate" ]]; then
                found=1
                break
            fi
        done

        if (( ! found )); then
            rm -f -- "$file"
        fi
    done < <(find "$dir" -type f -print0)

    find "$dir" -depth -type d -empty -delete
}

# Delete immediate subdirectories except those named in the remaining arguments.
keep_only_directories() {
    local dir="$1"
    shift
    local keep=("$@")
    local child name candidate found

    while IFS= read -r -d '' child; do
        name="${child##*/}"
        found=0
        for candidate in "${keep[@]}"; do
            if [[ "$name" == "$candidate" ]]; then
                found=1
                break
            fi
        done

        if (( ! found )); then
            rm -rf -- "$child"
        fi
    done < <(find "$dir" -mindepth 1 -maxdepth 1 -type d -print0)
}

# Keep only library families and floating-point ABIs reachable from build.rs.
prune_library_matrix() {
    local root="$1"
    local lib_dir family_dir family

    while IFS= read -r -d '' lib_dir; do
        while IFS= read -r -d '' family_dir; do
            family="${family_dir##*/}"
            case "$family" in
                nrf52 | nrf53 | nrf54h | nrf54l | nrf54l_ns | nrf54lm) ;;
                *) rm -rf -- "$family_dir" ;;
            esac
        done < <(find "$lib_dir" -mindepth 1 -maxdepth 1 -type d -print0)
    done < <(find "$root" -type d -name lib -print0)

    find "$root" -type d -name softfp-float -prune -exec rm -rf -- {} +
    find "$root" -type f -name manifest.yaml -delete
}

CMSIS5_FILES=(
    LICENSE.txt
    CMSIS/Core/Include/cmsis_compiler.h
    CMSIS/Core/Include/cmsis_gcc.h
    CMSIS/Core/Include/cmsis_version.h
    CMSIS/Core/Include/core_cm33.h
    CMSIS/Core/Include/core_cm4.h
    CMSIS/Core/Include/mpu_armv7.h
    CMSIS/Core/Include/mpu_armv8.h
)

NRFX_MDK_DIRECTORIES=(
    common
    nrf52
    nrf53
    nrf54h
    nrf54l
)

# ── CMSIS_5: only the Core headers + license ──
sparse_clone "$CMSIS5_REPO" "$CMSIS5_REF" "$THIRD_PARTY/arm/CMSIS_5" \
    CMSIS/Core/Include
keep_only_paths "$THIRD_PARTY/arm/CMSIS_5" "${CMSIS5_FILES[@]}"

# ── nrfx: specific headers from bsp (mdk/soc), drivers, hal, haly, templates ──
# Note: nrfx 4.x moved the MDK under bsp/stable/mdk and added the BSP layer
# (bsp/stable/nrfx_bsp.h, bsp/stable/soc/*) pulled in by drivers/nrfx_common.h.
sparse_clone "$NRFX_REPO" "$NRFX_REF" "$THIRD_PARTY/nordic/nrfx" \
    bsp drivers hal haly templates

# Remove top-level files except LICENSE and nrfx.h
keep_only "$THIRD_PARTY/nordic/nrfx" LICENSE nrfx.h
rm -rf "$THIRD_PARTY/nordic/nrfx/drivers/include" \
       "$THIRD_PARTY/nordic/nrfx/drivers/src"
keep_only "$THIRD_PARTY/nordic/nrfx/hal" nrf_common.h nrf_clock.h
keep_only "$THIRD_PARTY/nordic/nrfx/haly" nrfy_common.h
keep_only "$THIRD_PARTY/nordic/nrfx/templates" nrfx_glue.h
keep_only "$THIRD_PARTY/nordic/nrfx/bsp/stable" nrfx_bsp.h nrfx_ext.h
rm -rf "$THIRD_PARTY/nordic/nrfx/bsp/stable/templates"
keep_only_directories "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk" \
    "${NRFX_MDK_DIRECTORIES[@]}"

# The sys crates use only a single chip from each architecture
keep_only_directories "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf52" nrf52840
keep_only_directories "$THIRD_PARTY/nordic/nrfx/bsp/stable/mdk/nrf54l" \
    nrf54l15 nrf54lm20a

find "$THIRD_PARTY/nordic/nrfx" -type f \
    \( -name '*.svd' -o -name '*.c' -o -name '*.S' -o -name '*.s' \
       -o -name '*.asm' -o -name '*.ld' -o -name '*.x' \) \
    -delete
find "$THIRD_PARTY/nordic/nrfx" -depth -type d -empty -delete

# ── nrfxlib: mpsl + softdevice_controller headers and libs ──
sparse_clone "$NRFXLIB_REPO" "$NRFXLIB_REF" "$THIRD_PARTY/nordic/nrfxlib" \
    mpsl/include \
    mpsl/lib/nrf52 mpsl/lib/nrf53 mpsl/lib/nrf54l mpsl/lib/nrf54l_ns mpsl/lib/nrf54lm mpsl/lib/nrf54h \
    mpsl/fem/include mpsl/fem/common \
    mpsl/fem/simple_gpio mpsl/fem/nrf21540_gpio mpsl/fem/nrf21540_gpio_spi \
    softdevice_controller/include \
    softdevice_controller/lib/nrf52 softdevice_controller/lib/nrf53 \
    softdevice_controller/lib/nrf54l softdevice_controller/lib/nrf54l_ns softdevice_controller/lib/nrf54h \
    softdevice_controller/lib/nrf54lm

prune_library_matrix "$THIRD_PARTY/nordic/nrfxlib/mpsl"
prune_library_matrix "$THIRD_PARTY/nordic/nrfxlib/softdevice_controller"
