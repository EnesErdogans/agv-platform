#!/bin/sh
# Kaynak koddan calistirma (Node.js 18+ gerekir)
cd "$(dirname "$0")"
[ -d node_modules ] || npm install
npm start
