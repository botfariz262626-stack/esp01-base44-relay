// =============================================
// RELAY SERVER untuk ESP01 → Base44
// Deploy gratis di Render.com (Node.js web service)
// =============================================

const express = require('express');
const app = express();
const PORT = process.env.PORT || 3000;

// ==== KONFIGURASI — isi setelah deploy ====
const BASE44_APP_ID  = process.env.BASE44_APP_ID  || 'YOUR_APP_ID';
const BASE44_API_KEY = process.env.BASE44_API_KEY  || 'YOUR_API_KEY';
const BASE44_BASE    = `https://app.base44.com/api/apps/${BASE44_APP_ID}/entities`;

app.use(express.json());

// ── Helper: headers untuk Base44 ──────────────────
function b44Headers() {
  return {
    'Content-Type': 'application/json',
    'api_key': BASE44_API_KEY
  };
}

// ── Helper: fetch (Node 18+ native fetch) ────────
async function b44Fetch(path, method = 'GET', body = null) {
  const opts = { method, headers: b44Headers() };
  if (body) opts.body = JSON.stringify(body);
  const res = await fetch(`${BASE44_BASE}${path}`, opts);
  const text = await res.text();
  try { return JSON.parse(text); }
  catch { return { raw: text }; }
}

// ============================================================
// POST /data  — terima data dari ESP01, simpan ke SensorData
// Query params: pv, sp, out, mode, run, ah, al, kp, ki, kd, pm, rl
// ============================================================
app.get('/data', async (req, res) => {
  try {
    const q = req.query;
    const record = {
      pv:       parseFloat(q.pv   ?? 0),
      sp:       parseFloat(q.sp   ?? 0),
      out:      parseInt(  q.out  ?? 0),
      mode:     q.mode ?? 'P',
      run:      parseInt(  q.run  ?? 0),
      alarm_hi: parseInt(  q.ah   ?? 0),
      alarm_lo: parseInt(  q.al   ?? 0),
      kp:       parseFloat(q.kp   ?? 0),
      ki:       parseFloat(q.ki   ?? 0),
      kd:       parseFloat(q.kd   ?? 0),
      pmax:     parseInt(  q.pm   ?? 240),
      rlim:     parseInt(  q.rl   ?? 25)
    };

    const result = await b44Fetch('/SensorData', 'POST', record);
    console.log('[DATA] saved:', JSON.stringify(record));
    res.json({ ok: true, id: result.id });
  } catch (e) {
    console.error('[DATA ERROR]', e.message);
    res.status(500).json({ ok: false, err: e.message });
  }
});

// ============================================================
// GET /command  — ambil command terbaru yg belum dieksekusi
// ESP01 polling endpoint ini setiap CMD_INTERVAL
// ============================================================
app.get('/command', async (req, res) => {
  try {
    // Ambil command dengan executed=false, terbaru duluan
    const list = await b44Fetch('/Command?executed=false&sort=-created_date&limit=1');
    
    let cmd = null;
    if (Array.isArray(list) && list.length > 0) {
      cmd = list[0];
      // Mark as executed
      await b44Fetch(`/Command/${cmd.id}`, 'PUT', { executed: true });
      console.log('[CMD] dispatched:', cmd.id);
    }

    res.json({ cmd });
  } catch (e) {
    console.error('[CMD ERROR]', e.message);
    res.status(500).json({ cmd: null, err: e.message });
  }
});

// Health check
app.get('/', (req, res) => res.send('ESP01 Relay Server OK'));

app.listen(PORT, () => console.log(`Relay running on port ${PORT}`));
