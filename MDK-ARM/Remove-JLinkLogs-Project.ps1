# 自动查找并删除 D:\Project 目录下所有 JLinkLog.txt 文件
# 运行前先列出待删除文件，确认后再删除，避免误删

$root = "D:\Project"
$files = Get-ChildItem -Path $root -Recurse -Filter JLinkLog.txt -File -ErrorAction SilentlyContinue

if ($files.Count -eq 0) {
    Write-Host "在 $root 下未找到 JLinkLog.txt 文件" -ForegroundColor Green
    exit 0
}

Write-Host "找到 $($files.Count) 个 JLinkLog.txt:" -ForegroundColor Yellow
$files | ForEach-Object { Write-Host "  $($_.FullName)" }

$confirm = Read-Host "是否全部删除? (Y/N)"
if ($confirm -eq 'Y' -or $confirm -eq 'y') {
    $files | Remove-Item -Force
    Write-Host "已删除" -ForegroundColor Green
} else {
    Write-Host "已取消" -ForegroundColor Cyan
}
