@echo off
rem Kaynak koddan calistirma (Node.js 18+ gerekir)
cd /d "%~dp0"
if not exist node_modules call npm install
call npm start
