$ErrorActionPreference = 'Stop'
$path = 'C:\Users\krist\AppData\Roaming\Code\User\workspaceStorage\49031a2baa77a09df2cc745f3a4ca2cf\GitHub.copilot-chat\chat-session-resources\136dd418-46d9-4b8d-84b9-bfc5b986a23b\call_1R7exOTZngTa2NrRPMD0AML5__vscode-1786372450053\content.txt'
Select-String -Path $path -Pattern 'METASENSE_CAN_|METASENSE_LEAF_CAN_|CPPDEFINES' | Select-Object -First 300 | ForEach-Object { $_.Line }
