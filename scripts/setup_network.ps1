# Livox Mid-40 network setup (run in an *Administrator* PowerShell)
#
#   powershell -ExecutionPolicy Bypass -File scripts\setup_network.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\setup_network.ps1 -Revert
#
# Gives the wired adapter a static IP on the Mid-40 default subnet
# (LiDAR = 192.168.1.1xx) and allows inbound UDP on the ports this project uses.

param(
    [string]$Adapter = "Ethernet",
    [string]$HostIP = "192.168.1.50",
    [int]$Prefix = 24,
    [switch]$Revert
)

$ErrorActionPreference = "Stop"
$ruleName = "Livox Mid-40 (UDP 55000,56000-56002)"

if ($Revert) {
    Set-NetIPInterface -InterfaceAlias $Adapter -Dhcp Enabled
    Get-NetIPAddress -InterfaceAlias $Adapter -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object PrefixOrigin -eq Manual | Remove-NetIPAddress -Confirm:$false
    Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue | Remove-NetFirewallRule
    Write-Host "Reverted $Adapter to DHCP and removed firewall rule."
    exit 0
}

Write-Host "Setting $Adapter -> $HostIP/$Prefix"
Set-NetIPInterface -InterfaceAlias $Adapter -Dhcp Disabled
Get-NetIPAddress -InterfaceAlias $Adapter -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Remove-NetIPAddress -Confirm:$false
New-NetIPAddress -InterfaceAlias $Adapter -IPAddress $HostIP -PrefixLength $Prefix | Out-Null

if (-not (Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Protocol UDP `
        -LocalPort 55000,56000,56001,56002 -Action Allow -Profile Any | Out-Null
    Write-Host "Added firewall rule '$ruleName'"
}

Get-NetIPAddress -InterfaceAlias $Adapter -AddressFamily IPv4 | Format-Table InterfaceAlias, IPAddress, PrefixLength
Write-Host "Done. Now run:  python tools\livox_monitor.py --discover"
