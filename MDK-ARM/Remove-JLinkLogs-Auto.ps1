# 递归查找并自动删除当前目录下所有 JLinkLog.txt（不确认）

$files = Get-ChildItem -Recurse -Filter JLinkLog.txt -File

if ($files.Count -eq 0) {
    Write-Host "未找到 JLinkLog.txt 文件" -ForegroundColor Green
    exit 0
}

$files | Remove-Item -Force
Write-Host "已删除 $($files.Count) 个 JLinkLog.txt:" -ForegroundColor Green
$files | ForEach-Object { Write-Host "  $($_.FullName)" }
