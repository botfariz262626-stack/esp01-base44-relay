# ESP01 → Base44 Relay Server

Deploy gratis di Render.com. Node.js 18+.

## Cara Deploy di Render.com

1. Push folder ini ke GitHub repo baru
2. Buka https://render.com → New → Web Service
3. Connect repo GitHub kamu
4. Settings:
   - Name: esp01-relay (bebas)
   - Region: Singapore (paling deket Indonesia)
   - Branch: main
   - Runtime: Node
   - Build Command: npm install
   - Start Command: npm start
5. Add Environment Variables:
   - BASE44_APP_ID  = [app id dari Base44]
   - BASE44_API_KEY = [api key dari Base44 settings]
6. Klik Deploy!

URL relay kamu: https://esp01-relay.onrender.com (sesuai nama)

## Endpoints

GET /data?pv=75.2&sp=80&out=180&mode=P&run=1&ah=0&al=0&kp=5&ki=0.2&kd=0&pm=240&rl=25
→ Simpan data sensor ke Base44

GET /command
→ Ambil command terbaru dari dashboard (jika ada)
Response: {"cmd": {"cmd_sp": 60, "cmd_mode": "P", ...}} atau {"cmd": null}

GET /
→ Health check
