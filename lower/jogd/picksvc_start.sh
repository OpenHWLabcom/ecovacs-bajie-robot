#!/bin/bash
export PATH=/usr/bin:/bin
pkill -f "jogd/picksvc.py" 2>/dev/null
sleep 1
setsid nohup bash /home/Agi/jogd/picksvc_run.sh > /tmp/picksvc.log 2>&1 < /dev/null &
sleep 1
echo started
