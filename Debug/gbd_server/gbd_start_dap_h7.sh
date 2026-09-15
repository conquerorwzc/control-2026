#!/bin/bash

# ====== Config ======
INTERFACE="interface/cmsis-dap.cfg"
TARGET="target/stm32h7x.cfg"
LOGFILE="/tmp/openocd_h7.tmp"
CHECK_INTERVAL=2
# ==================

# Function to kill openocd
kill_openocd() {
    pkill -f "openocd -f $INTERFACE -f $TARGET" >/dev/null 2>&1
    sleep 1
}

# Cleanup on exit
trap 'kill_openocd; exit 0' SIGINT SIGTERM

kill_openocd

while true; do
    > "$LOGFILE"

    echo ""
    echo "[$(date +%T)] ============================================"
    echo "[$(date +%T)]  Starting OpenOCD - STM32H7"
    echo "[$(date +%T)]  GDB: target remote :3333"
    echo "[$(date +%T)]  Ctrl+C to quit"
    echo "[$(date +%T)] ============================================"
    echo ""

    openocd -f $INTERFACE -f $TARGET > "$LOGFILE" 2>&1 &
    OPENOCD_PID=$!

    echo "[$(date +%T)] Waiting for OpenOCD to start..."
    WAIT_COUNT=0
    CONNECTED=0

    while [ $WAIT_COUNT -lt 15 ]; do
        sleep 1
        WAIT_COUNT=$((WAIT_COUNT + 1))

        if ! kill -0 $OPENOCD_PID 2>/dev/null; then
            echo ""
            echo "[$(date +%T)] OpenOCD failed to start!"
            echo "[$(date +%T)] ---------- Log ----------"
            cat "$LOGFILE" 2>/dev/null
            echo "[$(date +%T)] -------------------------"
            echo "[$(date +%T)] Retry in 5s..."
            sleep 5
            break
        fi

        if grep -qi "Listening on port" "$LOGFILE"; then
            echo "[$(date +%T)] *** Connected successfully! ***"
            echo ""
            echo "[$(date +%T)] --- OpenOCD Log ---"
            cat "$LOGFILE" 2>/dev/null
            echo ""
            echo "[$(date +%T)] --- Monitoring ---"
            CONNECTED=1
            break
        fi
    done

    if [ $CONNECTED -eq 1 ]; then
        while true; do
            sleep $CHECK_INTERVAL
            if ! kill -0 $OPENOCD_PID 2>/dev/null; then
                echo ""
                echo "[$(date +%T)] OpenOCD process exited!"
                break
            fi

            if grep -qi "error writing data" "$LOGFILE"; then
                echo ""
                echo "[$(date +%T)] !! DAP-Link disconnected !!"
                echo "[$(date +%T)] Killing OpenOCD..."
                kill_openocd
                break
            fi
        done
    elif [ $WAIT_COUNT -ge 15 ]; then
        echo "[$(date +%T)] Startup timeout. Retrying..."
        kill_openocd
    fi

    echo ""
    echo "[$(date +%T)] ========================================"
    echo "[$(date +%T)]  Connection lost. Retry in 5s..."
    echo "[$(date +%T)]  (Plug DAP-Link back in)"
    echo "[$(date +%T)]  Ctrl+C to quit."
    echo "[$(date +%T)] ========================================"
    echo ""
    sleep 5
done
