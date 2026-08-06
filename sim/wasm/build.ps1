# 펌웨어 oled_ui.c 를 WASM 으로 빌드 → sim\oled_sim.js (+ .wasm).
#  펌웨어(oled_ui.c) 수정 후 이 스크립트만 재실행하면 웹 시뮬에 반영된다.
$ErrorActionPreference = "Stop"
$root = "D:\dev\workspaces\somfy-blinds-things-by-claude"
$M = "$root\main"; $S = "$root\sim\wasm"
$emcc = "D:\emsdk\upstream\emscripten\emcc.py"
$py = (Get-Command python).Source

& $py $emcc `
  "$M\oled_ui.c" "$S\glue.c" `
  -I"$S\inc" -I"$M" `
  -DOLED_SIM `
  -O2 -s ALLOW_MEMORY_GROWTH=1 `
  -s "EXPORTED_FUNCTIONS=['_sim_init','_sim_action','_sim_action_end','_sim_select','_sim_freq','_sim_tick','_sim_fb','_sim_pw','_sim_ph','_main']" `
  -s "EXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8']" `
  -s MODULARIZE=1 -s "EXPORT_NAME=OledSim" `
  -o "$root\sim\oled_sim.js"

if ($LASTEXITCODE -eq 0) { Write-Host "BUILD OK -> sim\oled_sim.js" } else { Write-Host "BUILD FAILED ($LASTEXITCODE)" }
