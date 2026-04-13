import express from "express";
import fetch from "node-fetch";

const app = express();
const PORT = process.env.PORT || 3000;

const B44_BASE = "https://base44.app/api/apps/69dcf71c53c101c1159af4b0/entities";
const headers = {
  "Content-Type": "application/json",
  "api_key": "619e574ca3d44fd880966002af9810c2",
};

// ESP kirim data sensor → simpan ke SensorData
app.get("/functions/receiveData", async (req, res) => {
  const q = req.query;
  const body = {
    pv:       parseFloat(q.pv)  || 0,
    sp:       parseFloat(q.sp)  || 0,
    output:   parseInt(q.out)   || 0,
    mode:     q.mode === "P" ? "PID" : "MANUAL",
    running:  q.run === "1",
    alarm_hi: q.ah === "1",
    alarm_lo: q.al === "1",
    kp:       parseFloat(q.kp)  || 0,
    ki:       parseFloat(q.ki)  || 0,
    kd:       parseFloat(q.kd)  || 0,
    pmax:     parseInt(q.pm)    || 240,
    rlim:     parseInt(q.rl)    || 25,
  };

  await fetch(`${B44_BASE}/SensorData`, {
    method: "POST",
    headers,
    body: JSON.stringify(body),
  });

  res.json({ ok: true });
});

// ESP poll command → ambil command terbaru dari DeviceCommand
app.get("/functions/getCommand", async (req, res) => {
  const resp = await fetch(
    `${B44_BASE}/DeviceCommand?consumed=false&_sort=-created_date&_limit=1`,
    { headers }
  );
  const list = await resp.json();

  if (!list || list.length === 0) {
    return res.json({ cmd: null });
  }

  const cmd = list[0];

  // Mark consumed supaya tidak dieksekusi lagi
  await fetch(`${B44_BASE}/DeviceCommand/${cmd.id}`, {
    method: "PUT",
    headers,
    body: JSON.stringify({ consumed: true }),
  });

  res.json({ cmd });
});

app.listen(PORT, () => console.log(`Relay on port ${PORT}`));
Yang perlu ada di Railway repo:

package.json:

{
  "type": "module",
  "scripts": { "start": "node index.js" },
  "dependencies": {
    "express": "^4.18.0",
    "node-fetch": "^3.3.0"
  }
}
