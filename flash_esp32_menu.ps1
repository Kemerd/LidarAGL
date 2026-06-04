# ============================================================================
#  flash_esp32_menu.ps1  -  Interactive arrow-key COM-port picker.
#
#  Driven by flash_esp32.bat. We render the menu with Write-Host (straight to
#  the console), and emit ONLY the chosen "COMx" string on stdout via
#  Write-Output - so the calling batch script can capture it with a FOR /F loop
#  without the menu chrome leaking into the variable.
#
#  Controls:  Up / Down  - move      Enter - confirm      Esc - cancel
#
#  The last-confirmed port is remembered in flash_esp32_default.txt (next to
#  this script, git-ignored). On launch that port is pre-highlighted, so just
#  pressing Enter re-flashes the same device.
# ============================================================================
param(
    # Optional explicit port (e.g. "COM7"). When supplied we skip the menu
    # entirely and hand it straight back - lets `flash_esp32.bat COM7` work.
    [string]$Port = ''
)

# --- Where we persist the remembered default (sibling of this script). -------
$defaultFile = Join-Path $PSScriptRoot 'flash_esp32_default.txt'

# ----------------------------------------------------------------------------
#  Short-circuit: caller passed an explicit port -> remember it and return it.
# ----------------------------------------------------------------------------
if ($Port -ne '') {
    Set-Content -Path $defaultFile -Value $Port -Encoding ASCII
    Write-Output $Port
    exit 0
}

# ----------------------------------------------------------------------------
#  Enumerate every PnP device that exposes a (COMx) endpoint, sorted by number.
#  Each entry keeps the bare port ("COM4") and the friendly name so you can
#  tell the ESP32-S3 (USB-Serial-JTAG / VID_303A) apart from random adapters.
# ----------------------------------------------------------------------------
$ports = @(
    Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Name -match '\(COM\d+\)' } |
        ForEach-Object {
            if ($_.Name -match '\((COM\d+)\)') {
                [pscustomobject]@{
                    Port = $matches[1]
                    Num  = [int]($matches[1] -replace '\D')
                    Name = $_.Name
                }
            }
        } |
        Sort-Object Num
)

# --- Nothing plugged in? Tell the human, return nothing. --------------------
if ($ports.Count -eq 0) {
    Write-Host ''
    Write-Host '  No COM ports found. Plug the ESP32-S3 in and re-run.' -ForegroundColor Yellow
    Write-Host ''
    exit 1
}

# ----------------------------------------------------------------------------
#  Start the cursor on the previously-saved default if it's still present;
#  otherwise default to the first entry.
# ----------------------------------------------------------------------------
$selected = 0
if (Test-Path $defaultFile) {
    $saved = (Get-Content $defaultFile -Raw).Trim()
    for ($i = 0; $i -lt $ports.Count; $i++) {
        if ($ports[$i].Port -eq $saved) { $selected = $i; break }
    }
}

# ----------------------------------------------------------------------------
#  Draw the menu in place. We stash the top row once, then re-home the cursor
#  on every keystroke and overwrite - no flicker, no scrolling wall of menus.
# ----------------------------------------------------------------------------
function Show-Menu {
    param([int]$Top)

    [Console]::SetCursorPosition(0, $Top)
    Write-Host ''
    Write-Host '  Select the ESP32 port  ' -ForegroundColor Cyan -NoNewline
    Write-Host '(Up/Down, Enter, Esc)'      -ForegroundColor DarkGray
    Write-Host '  ----------------------------------------------'

    for ($i = 0; $i -lt $ports.Count; $i++) {
        $p   = $ports[$i]
        # Pad to a fixed width so a shorter line never leaves stale characters.
        $line = ('   {0,-6} {1}' -f $p.Port, $p.Name).PadRight([Console]::WindowWidth - 4)

        if ($i -eq $selected) {
            # Highlighted row: arrow + inverted colours so it pops.
            Write-Host (' > ' + $line.Substring(3)) -ForegroundColor Black -BackgroundColor Cyan
        } else {
            Write-Host $line
        }
    }
    Write-Host '  ----------------------------------------------'
}

# --- Make room, hide the caret, capture our anchor row. ---------------------
[Console]::CursorVisible = $false
# Print enough blank lines so SetCursorPosition never points past the buffer,
# then walk the cursor back up to where the menu should start drawing.
$menuHeight = $ports.Count + 5
for ($i = 0; $i -lt $menuHeight; $i++) { Write-Host '' }
$top = [Console]::CursorTop - $menuHeight
if ($top -lt 0) { $top = 0 }

# ----------------------------------------------------------------------------
#  Input loop: arrow keys move the highlight, Enter picks, Esc bails.
# ----------------------------------------------------------------------------
$confirmed = $null
while ($true) {
    Show-Menu -Top $top
    $key = [Console]::ReadKey($true)

    switch ($key.Key) {
        'UpArrow'   { if ($selected -gt 0)                 { $selected-- } }
        'DownArrow' { if ($selected -lt $ports.Count - 1)  { $selected++ } }
        'Enter'     { $confirmed = $ports[$selected].Port; break }
        'Escape'    { $confirmed = $null;                  break }
    }
}

# --- Restore the caret and move below the menu so later output looks clean. --
[Console]::CursorVisible = $true
[Console]::SetCursorPosition(0, $top + $menuHeight)

# --- Cancelled: emit nothing, non-zero exit. --------------------------------
if ($null -eq $confirmed) {
    Write-Host '  Cancelled.' -ForegroundColor Yellow
    exit 1
}

# --- Persist as the new default and hand the bare port back to the batch. ---
Set-Content -Path $defaultFile -Value $confirmed -Encoding ASCII
Write-Output $confirmed
exit 0
