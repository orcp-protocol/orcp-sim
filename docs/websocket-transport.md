# WebSocket transport contract

How a browser host (e.g. the MC1 web configurator) connects to the simulator for
hardware-free testing.

## Why this exists

The web configurator talks to real boards over the **Web Serial API** (USB CDC,
115200 8N1). Browsers cannot open a Unix PTY through Web Serial, so they can't
reach the simulator's default serial mode. ORCP is transport-agnostic — the same
ASCII line protocol runs over any byte stream — so the simulator can also serve
over a **WebSocket**, which browsers speak natively.

The configurator only needs a small transport abstraction: **Web Serial for
hardware, WebSocket for the simulator.** Everything above the byte layer (line
framing, `INFO`/`STATUS` parsing, the config keys, push handling) is identical.

## Starting the simulator

```bash
pip install 'orcp-sim[web]'
orcp-sim --ws 8765            # serve on ws://localhost:8765
orcp-sim --ws 8765 --level 1 # emulate a Level 1 controller
```

## The contract

- **Endpoint:** `ws://localhost:<PORT>` (the port passed to `--ws`).
- **Framing:** the WebSocket carries the **identical byte stream** you would read
  from the serial port. Use **text frames**. A frame contains one or more
  `\n`-terminated ORCP lines. The client MUST buffer payloads and split on `\n` —
  exactly the same line-assembly logic as reading the serial port. Do not assume
  one frame == one line.
- **Client → server:** command lines, each `\n`-terminated (e.g. `INFO\n`,
  `SET pid.kp=0.080\n`).
- **Server → client:**
  - Command responses: `OK ...\n` or `ERR <code> <message>\n`.
  - Unsolicited pushes: lines beginning with `! ` (`! FAULT <code>`,
    `! WARN ...`, `! STREAM ...`), arriving at any time.
- **Encoding:** ASCII. **Baud rate is irrelevant** over WebSocket — ignore it in
  simulator mode.
- **Identity:** as with serial, read the board UUID from `INFO` (`uuid=`).

In short: drop the WebSocket's incoming bytes into the same parser you already
feed from the serial reader, and write command bytes to the socket instead of the
serial port. Nothing else changes.

## Reference: a transport interface for the configurator

```js
// Common interface the rest of the app uses, regardless of transport.
//   connect(), send(line), onLine(cb), close()

class WebSocketTransport {
  constructor(url) { this.url = url; this._buf = ""; this._cb = null; }

  connect() {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.url);
      this.ws.onopen = () => resolve();
      this.ws.onerror = (e) => reject(e);
      this.ws.onmessage = (ev) => {
        this._buf += ev.data;                 // may hold partial / multiple lines
        let i;
        while ((i = this._buf.indexOf("\n")) >= 0) {
          const line = this._buf.slice(0, i).replace(/\r$/, "");
          this._buf = this._buf.slice(i + 1);
          if (line && this._cb) this._cb(line); // "OK ...", "ERR ...", or "! ..."
        }
      };
    });
  }

  send(line) { this.ws.send(line.endsWith("\n") ? line : line + "\n"); }
  onLine(cb) { this._cb = cb; }
  close()    { this.ws.close(); }
}

// A WebSerialTransport implementing the same connect()/send()/onLine()/close()
// wraps navigator.serial; the app picks one based on a "Connect to hardware" vs
// "Connect to simulator" choice. The line-handling callback is shared.
```

## Notes

- `localhost` WebSocket connections are not subject to Web Serial's user-gesture
  permission prompt, which makes the simulator convenient for automated UI tests.
- Mixed-content: a configurator served over **https** cannot open a plain `ws://`
  endpoint. For local dev, serve the configurator over `http://localhost` (allowed
  to reach `ws://localhost`), or run it from a file/dev server over http.
- The simulator currently binds `localhost` only (not exposed to the network).
