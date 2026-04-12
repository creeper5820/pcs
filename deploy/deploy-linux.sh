#!/usr/bin/env bash
set -euo pipefail

readonly APPIMAGETOOL_URL="https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage"
readonly MATERIAL_ICONS_URL="https://github.com/google/material-design-icons/raw/master/font/MaterialIconsRound-Regular.otf"

readonly QT_PLUGIN_FALLBACK_DIRS=(
	"/usr/lib/qt6/plugins"
	"/usr/lib64/qt6/plugins"
	"/usr/lib/x86_64-linux-gnu/qt6/plugins"
	"/usr/lib/qt5/plugins"
	"/usr/lib/x86_64-linux-gnu/qt5/plugins"
)

readonly QT_PLUGIN_PATTERNS=(
	"platforms/libqxcb.so"
	"platforminputcontexts/libcomposeplatforminputcontextplugin.so"
	"xcbglintegrations/libqxcb-egl-integration.so"
	"xcbglintegrations/libqxcb-glx-integration.so"
	"imageformats/libqico.so"
	"imageformats/libqjpeg.so"
)

readonly SKIPPED_LIBRARY_PATTERNS=(
	"linux-vdso.so*"
	"ld-linux*.so*"
	"libc.so*"
	"libm.so*"
	"libpthread.so*"
	"librt.so*"
	"libdl.so*"
	"libresolv.so*"
	"libnss_*.so*"
	"libutil.so*"
	"libanl.so*"
	"libGL.so*"
	"libGLX.so*"
	"libGLdispatch.so*"
	"libOpenGL.so*"
	"libEGL.so*"
	"libdrm.so*"
	"libgbm.so*"
	"libwayland-*.so*"
)

readonly DEJAVU_FONT_FILES=(
	"DejaVuSans.ttf"
	"DejaVuSans-Bold.ttf"
	"DejaVuSansMono.ttf"
	"DejaVuSansMono-Bold.ttf"
	"DejaVuSerif.ttf"
	"DejaVuSerif-Bold.ttf"
)

usage() {
	printf 'Usage: %s <build-dir> <target> [release-dir]\n' "$0" >&2
}

die() {
	printf 'error: %s\n' "$1" >&2
	exit 1
}

download_file() {
	local url="$1"
	local dest="$2"
	local tmp_path="${dest}.tmp"

	rm -f "${tmp_path}"

	if command -v curl >/dev/null 2>&1; then
		if curl --http1.1 -L --fail --retry 8 --retry-delay 2 --retry-all-errors "${url}" -o "${tmp_path}"; then
			mv "${tmp_path}" "${dest}"
			return 0
		fi
		rm -f "${tmp_path}"
	fi

	if command -v wget >/dev/null 2>&1; then
		wget --tries=8 --waitretry=2 -O "${tmp_path}" "${url}"
		mv "${tmp_path}" "${dest}"
		return 0
	fi

	die "curl or wget is required to download ${url}"
}

resolve_release_dir() {
	local requested_dir="$1"

	mkdir -p "$(dirname "${requested_dir}")"
	printf '%s/%s\n' \
		"$(cd "$(dirname "${requested_dir}")" && pwd)" \
		"$(basename "${requested_dir}")"
}

locate_qt_plugin_dir() {
	local queried_path=""
	local candidate=""

	if [[ -n "${QT_PLUGIN_DIR:-}" && -d "${QT_PLUGIN_DIR}" ]]; then
		printf '%s\n' "${QT_PLUGIN_DIR}"
		return 0
	fi

	for queried_path in \
		"$(qmake6 -query QT_INSTALL_PLUGINS 2>/dev/null || true)" \
		"$(qtpaths6 --plugin-dir 2>/dev/null || true)" \
		"$(qmake -query QT_INSTALL_PLUGINS 2>/dev/null || true)" \
		"$(qtpaths --plugin-dir 2>/dev/null || true)"; do
		if [[ -n "${queried_path}" && -d "${queried_path}" ]]; then
			printf '%s\n' "${queried_path}"
			return 0
		fi
	done

	for candidate in "${QT_PLUGIN_FALLBACK_DIRS[@]}"; do
		if [[ -d "${candidate}" ]]; then
			printf '%s\n' "${candidate}"
			return 0
		fi
	done

	return 1
}

collect_linked_libraries() {
	local inspect_path="$1"
	local line=""
	local first=""
	local second=""
	local third=""
	local _rest=""

	while IFS= read -r line; do
		read -r first second third _rest <<<"${line}"

		if [[ "${second}" == "=>" && "${third}" == /* ]]; then
			printf '%s\n' "${third}"
			continue
		fi

		if [[ "${first}" == /* ]]; then
			printf '%s\n' "${first}"
		fi
	done < <(ldd "${inspect_path}" 2>/dev/null || true)
}

should_skip_library() {
	local so_name="$1"
	local pattern=""

	for pattern in "${SKIPPED_LIBRARY_PATTERNS[@]}"; do
		if [[ "${so_name}" == ${pattern} ]]; then
			return 0
		fi
	done

	return 1
}

ensure_appimagetool() {
	local tool_path="${build_dir}/appimagetool-official"

	if command -v appimagetool >/dev/null 2>&1; then
		command -v appimagetool
		return 0
	fi

	if [[ "${appimage_arch}" != "x86_64" ]]; then
		die "appimagetool is required on PATH for architecture ${appimage_arch}"
	fi

	if [[ ! -x "${tool_path}" ]]; then
		printf 'downloading appimagetool...\n' >&2
		download_file "${APPIMAGETOOL_URL}" "${tool_path}"
		chmod +x "${tool_path}"
	fi

	printf '%s\n' "${tool_path}"
}

ensure_material_icons_round_font() {
	local font_cache_dir="${build_dir}/fonts"
	local font_path="${font_cache_dir}/MaterialIconsRound-Regular.otf"

	mkdir -p "${font_cache_dir}"

	if [[ ! -s "${font_path}" ]]; then
		printf 'downloading Material Icons Round font...\n' >&2
		download_file "${MATERIAL_ICONS_URL}" "${font_path}"
	fi

	printf '%s\n' "${font_path}"
}

prepare_release_tree() {
	rm -rf "${release_dir}"

	mkdir -p \
		"${bin_dir}" \
		"${lib_dir}" \
		"${plugin_dir}" \
		"${font_dir}" \
		"${dejavu_font_dir}" \
		"${qt_font_dir}" \
		"${font_conf_dir}" \
		"${app_dir}/var/cache/fontconfig" \
		"${share_dir}/applications" \
		"${share_dir}/icons/hicolor/256x256/apps"
}

copy_runtime_assets() {
	cp -L "${binary_path}" "${bin_dir}/${target}"
	chmod +x "${bin_dir}/${target}"

	cp -L "${repo_dir}/deploy/pcs.desktop" "${share_dir}/applications/pcs.desktop"
	cp -L "${repo_dir}/deploy/pointcloudstudio.png" "${share_dir}/icons/hicolor/256x256/apps/pointcloudstudio.png"
	cp -L "${repo_dir}/deploy/pcs.desktop" "${app_dir}/pcs.desktop"
	cp -L "${repo_dir}/deploy/pointcloudstudio.png" "${app_dir}/pointcloudstudio.png"
}

copy_fonts() {
	local material_font_path=""
	local font_name=""
	local font_source=""

	material_font_path="$(ensure_material_icons_round_font)"
	cp -L "${material_font_path}" "${font_dir}/MaterialIconsRound-Regular.otf"
	cp -L "${material_font_path}" "${qt_font_dir}/MaterialIconsRound-Regular.otf"

	if [[ -d /usr/share/fonts/truetype/dejavu ]]; then
		for font_name in "${DEJAVU_FONT_FILES[@]}"; do
			font_source="/usr/share/fonts/truetype/dejavu/${font_name}"
			[[ -f "${font_source}" ]] || continue
			cp -L "${font_source}" "${dejavu_font_dir}/${font_name}"
			cp -L "${font_source}" "${qt_font_dir}/${font_name}"
		done
	fi
}

render_templates() {
	cp -L "${template_dir}/fonts.conf.in" "${font_conf_dir}/fonts.conf.in"
	sed "s|__TARGET__|${target}|g" "${template_dir}/AppRun.in" >"${app_dir}/AppRun"
	chmod +x "${app_dir}/AppRun"

	cat >"${release_dir}/${target}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
self_dir="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
exec "\${self_dir}/AppDir/AppRun" "\$@"
EOF
	chmod +x "${release_dir}/${target}"
}

copy_selected_qt_plugins() {
	local pattern=""
	local plugin_source=""
	local relative_path=""
	local destination_dir=""

	for pattern in "${QT_PLUGIN_PATTERNS[@]}"; do
		shopt -s nullglob
		for plugin_source in "${qt_plugin_src}/${pattern}"; do
			[[ -f "${plugin_source}" ]] || continue
			relative_path="${plugin_source#"${qt_plugin_src}/"}"
			destination_dir="${plugin_dir}/$(dirname "${relative_path}")"
			mkdir -p "${destination_dir}"
			cp -L "${plugin_source}" "${destination_dir}/$(basename "${plugin_source}")"
			dependency_roots+=("${plugin_source}")
		done
		shopt -u nullglob
	done
}

copy_runtime_libraries() {
	local -a queue=("${dependency_roots[@]}")
	local -A seen_library_names=()
	local -A visited_paths=()
	local current=""
	local current_real=""
	local dep_path=""
	local dep_name=""

	while ((${#queue[@]} > 0)); do
		current="${queue[0]}"
		queue=("${queue[@]:1}")

		current_real="$(readlink -f "${current}" 2>/dev/null || printf '%s' "${current}")"
		if [[ -n "${visited_paths[${current_real}]:-}" ]]; then
			continue
		fi
		visited_paths["${current_real}"]=1

		while IFS= read -r dep_path; do
			[[ -n "${dep_path}" ]] || continue

			dep_name="$(basename "${dep_path}")"
			if should_skip_library "${dep_name}"; then
				continue
			fi

			if [[ -z "${seen_library_names[${dep_name}]:-}" ]]; then
				cp -L "${dep_path}" "${lib_dir}/${dep_name}"
				seen_library_names["${dep_name}"]=1
			fi

			queue+=("${dep_path}")
		done < <(collect_linked_libraries "${current_real}")
	done
}

strip_bundle() {
	local strip_tool=""
	local path=""

	strip_tool="$(command -v strip || true)"
	if [[ -z "${strip_tool}" ]]; then
		return 0
	fi

	while IFS= read -r path; do
		[[ -f "${path}" ]] || continue
		"${strip_tool}" --strip-unneeded "${path}" 2>/dev/null || true
	done < <(find "${bin_dir}" "${lib_dir}" "${plugin_dir}" -type f -print)
}

package_appimage() {
	local appimagetool_path=""
	local extract_dir=""

	appimagetool_path="$(ensure_appimagetool)"
	rm -f "${appimage_path}"

	if "${appimagetool_path}" --appimage-version >/dev/null 2>&1; then
		extract_dir="$(mktemp -d "${build_dir}/appimagetool-extract.XXXXXX")"
		(
			cd "${extract_dir}"
			"${appimagetool_path}" --appimage-extract >/dev/null
			ARCH="${appimage_arch}" "${extract_dir}/squashfs-root/AppRun" \
				--no-appstream "${app_dir}" "${appimage_path}"
		)
		rm -rf "${extract_dir}"
	else
		ARCH="${appimage_arch}" "${appimagetool_path}" \
			--no-appstream "${app_dir}" "${appimage_path}"
	fi

	chmod +x "${appimage_path}"
}

if [[ $# -lt 2 || $# -gt 3 ]]; then
	usage
	exit 1
fi

build_dir="$1"
target="$2"
release_dir="${3:-$(pwd)/pcs-release}"

if [[ ! -d "${build_dir}" ]]; then
	die "build directory does not exist: ${build_dir}"
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/.." && pwd)"
template_dir="${script_dir}/template"
build_dir="$(cd "${build_dir}" && pwd)"
release_dir="$(resolve_release_dir "${release_dir}")"

app_dir="${release_dir}/AppDir"
usr_dir="${app_dir}/usr"
bin_dir="${usr_dir}/bin"
lib_dir="${usr_dir}/lib"
plugin_dir="${usr_dir}/plugins"
share_dir="${usr_dir}/share"
font_dir="${share_dir}/fonts/truetype/material-icons"
dejavu_font_dir="${share_dir}/fonts/truetype/dejavu"
qt_font_dir="${share_dir}/fonts/qt-bundle"
font_conf_dir="${app_dir}/etc/fonts"
binary_path="${build_dir}/${target}"
appimage_arch="$(uname -m)"
appimage_path="${release_dir}/${target}-${appimage_arch}.AppImage"

if [[ ! -x "${binary_path}" ]]; then
	die "target executable not found: ${binary_path}"
fi

qt_plugin_src="$(locate_qt_plugin_dir || true)"
if [[ -z "${qt_plugin_src}" ]]; then
	die "cannot locate Qt plugins directory"
fi

declare -a dependency_roots=("${bin_dir}/${target}")

prepare_release_tree
copy_runtime_assets
copy_fonts
render_templates
copy_selected_qt_plugins
copy_runtime_libraries
strip_bundle
package_appimage

printf 'deploy completed:\n'
printf '  appdir: %s\n' "${app_dir}"
printf '  launcher: %s\n' "${release_dir}/${target}"
printf '  appimage: %s\n' "${appimage_path}"
