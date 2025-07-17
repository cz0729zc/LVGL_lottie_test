@echo off

set /p commit_msg="Please enter your commit message: "

git add .
git commit -m "%commit_msg%"
git push

pause