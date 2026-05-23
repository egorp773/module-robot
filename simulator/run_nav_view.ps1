param(
  [string]$Mode = "--random",
  [string]$A = "123",
  [string]$B = "",
  [string]$Seed = ""
)

$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$toolchain = Join-Path $repoRoot ".tools\w64devkit\bin"
$source = Join-Path $repoRoot "navigation_core\demo_random_nav.cpp"
$include = Join-Path $repoRoot "rtk_firmware\lib\NavigationCore"
$outDir = Join-Path $repoRoot ".pio\build_root"
$exe = Join-Path $outDir "nav_demo.exe"
$csv = Join-Path $outDir "nav_demo_trace.csv"
$html = Join-Path $outDir "nav_demo_view.html"

if (-not (Test-Path (Join-Path $toolchain "g++.exe"))) {
  throw "g++ not found. Expected: $(Join-Path $toolchain 'g++.exe')"
}

New-Item -ItemType Directory -Force $outDir | Out-Null
$env:PATH = "$toolchain;$env:PATH"

g++ -std=c++17 -Wall -Wextra -I $include $source -o $exe

if ($Mode -eq "--target") {
  if ($B -eq "") {
    throw "Usage: .\run_nav_view.ps1 --target <x_m> <y_m> [seed]"
  }
  if ($Seed -eq "") {
    & $exe --target $A $B --csv $csv
  } else {
    & $exe --target $A $B $Seed --csv $csv
  }
} elseif ($Mode -eq "--random") {
  & $exe --random $A --csv $csv
} else {
  throw "Usage: .\run_nav_view.ps1 --random [seed] OR .\run_nav_view.ps1 --target <x_m> <y_m> [seed]"
}

$rows = Import-Csv $csv
if ($rows.Count -eq 0) {
  throw "CSV trace is empty: $csv"
}

$json = $rows | ConvertTo-Json -Compress

$template = @'
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>NavigationCore Simulation</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: "Segoe UI", Arial, sans-serif;
      background: #101214;
      color: #eef1f3;
    }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      grid-template-rows: auto 1fr auto;
    }
    header, footer {
      padding: 14px 18px;
      background: #171a1d;
      border-bottom: 1px solid #2a3035;
    }
    footer {
      border-top: 1px solid #2a3035;
      border-bottom: 0;
      color: #aeb6bd;
      font-size: 13px;
    }
    h1 {
      margin: 0 0 8px 0;
      font-size: 20px;
      font-weight: 650;
    }
    .stats {
      display: flex;
      flex-wrap: wrap;
      gap: 16px;
      color: #c8d0d6;
      font-size: 14px;
    }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1fr) 270px;
      min-height: 0;
    }
    canvas {
      width: 100%;
      height: 100%;
      display: block;
      background: #f5f7f8;
    }
    aside {
      padding: 14px;
      background: #171a1d;
      border-left: 1px solid #2a3035;
      display: grid;
      align-content: start;
      gap: 10px;
    }
    button {
      height: 34px;
      border: 1px solid #3f484f;
      background: #22282d;
      color: #eef1f3;
      border-radius: 6px;
      cursor: pointer;
    }
    input[type="range"] {
      width: 100%;
    }
    .metric {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 10px;
      padding: 8px 0;
      border-bottom: 1px solid #2a3035;
      font-size: 14px;
    }
    .metric span:first-child {
      color: #aeb6bd;
    }
    @media (max-width: 800px) {
      main {
        grid-template-columns: 1fr;
        grid-template-rows: minmax(420px, 1fr) auto;
      }
      aside {
        border-left: 0;
        border-top: 1px solid #2a3035;
      }
    }
  </style>
</head>
<body>
  <header>
    <h1>NavigationCore Simulation</h1>
    <div class="stats" id="summary"></div>
  </header>
  <main>
    <canvas id="canvas"></canvas>
    <aside>
      <button id="play">Pause</button>
      <input id="scrub" type="range" min="0" max="100" value="0">
      <div class="metric"><span>Time</span><strong id="time">0.0 s</strong></div>
      <div class="metric"><span>Position</span><strong id="pos">0, 0</strong></div>
      <div class="metric"><span>Heading</span><strong id="heading">0 deg</strong></div>
      <div class="metric"><span>Distance</span><strong id="dist">0 m</strong></div>
      <div class="metric"><span>Heading error</span><strong id="err">0 deg</strong></div>
      <div class="metric"><span>Motors</span><strong id="motors">0 / 0</strong></div>
      <div class="metric"><span>State</span><strong id="state">MOVING</strong></div>
    </aside>
  </main>
  <footer>Blue path is true robot position. Small gray dots are noisy GPS samples. Red dot is target.</footer>
  <script>
    const rows = __DATA__.map(r => ({
      t: Number(r.time_s),
      x: Number(r.true_x),
      y: Number(r.true_y),
      gpsX: Number(r.gps_x),
      gpsY: Number(r.gps_y),
      heading: Number(r.heading_deg),
      dist: Number(r.dist_m),
      err: Number(r.heading_error_deg),
      xtk: Number(r.cross_track_m),
      left: Number(r.left_cmd),
      right: Number(r.right_cmd),
      state: r.state,
      targetX: Number(r.target_x),
      targetY: Number(r.target_y)
    }));

    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    const play = document.getElementById('play');
    const scrub = document.getElementById('scrub');
    const ids = ['time', 'pos', 'heading', 'dist', 'err', 'motors', 'state'];
    const el = Object.fromEntries(ids.map(id => [id, document.getElementById(id)]));
    let idx = 0;
    let playing = true;
    let lastTs = 0;

    function resize() {
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.max(1, Math.floor(rect.width * dpr));
      canvas.height = Math.max(1, Math.floor(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }

    const xs = rows.flatMap(r => [r.x, r.gpsX, r.targetX, 0]);
    const ys = rows.flatMap(r => [r.y, r.gpsY, r.targetY, 0]);
    const bounds = {
      minX: Math.min(...xs) - 0.8,
      maxX: Math.max(...xs) + 0.8,
      minY: Math.min(...ys) - 0.8,
      maxY: Math.max(...ys) + 0.8
    };

    function project(x, y) {
      const rect = canvas.getBoundingClientRect();
      const pad = 36;
      const w = rect.width - pad * 2;
      const h = rect.height - pad * 2;
      const sx = w / Math.max(0.1, bounds.maxX - bounds.minX);
      const sy = h / Math.max(0.1, bounds.maxY - bounds.minY);
      const s = Math.min(sx, sy);
      const cx = (bounds.minX + bounds.maxX) / 2;
      const cy = (bounds.minY + bounds.maxY) / 2;
      return {
        x: rect.width / 2 + (x - cx) * s,
        y: rect.height / 2 - (y - cy) * s,
        s
      };
    }

    function drawGrid() {
      const rect = canvas.getBoundingClientRect();
      ctx.fillStyle = '#f5f7f8';
      ctx.fillRect(0, 0, rect.width, rect.height);
      ctx.strokeStyle = '#d7dde1';
      ctx.lineWidth = 1;
      for (let m = Math.floor(bounds.minX); m <= Math.ceil(bounds.maxX); m++) {
        const a = project(m, bounds.minY);
        const b = project(m, bounds.maxY);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
      for (let m = Math.floor(bounds.minY); m <= Math.ceil(bounds.maxY); m++) {
        const a = project(bounds.minX, m);
        const b = project(bounds.maxX, m);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
      const origin = project(0, 0);
      ctx.fillStyle = '#20262b';
      ctx.beginPath();
      ctx.arc(origin.x, origin.y, 4, 0, Math.PI * 2);
      ctx.fill();
    }

    function drawRobot(r) {
      const p = project(r.x, r.y);
      const angle = r.heading * Math.PI / 180;
      const size = Math.max(13, p.s * 0.18);
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(angle);
      ctx.fillStyle = '#1f7a8c';
      ctx.strokeStyle = '#0b3f4a';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, -size);
      ctx.lineTo(size * 0.65, size * 0.8);
      ctx.lineTo(0, size * 0.45);
      ctx.lineTo(-size * 0.65, size * 0.8);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    function draw() {
      if (!rows.length) return;
      drawGrid();
      const current = rows[idx];
      const target = project(current.targetX, current.targetY);

      ctx.strokeStyle = '#2066d1';
      ctx.lineWidth = 3;
      ctx.beginPath();
      rows.slice(0, idx + 1).forEach((r, i) => {
        const p = project(r.x, r.y);
        if (i === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();

      ctx.fillStyle = 'rgba(80, 86, 92, 0.35)';
      for (let i = 0; i <= idx; i += 4) {
        const p = project(rows[i].gpsX, rows[i].gpsY);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 2, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.fillStyle = '#d62828';
      ctx.beginPath();
      ctx.arc(target.x, target.y, 8, 0, Math.PI * 2);
      ctx.fill();

      drawRobot(current);

      el.time.textContent = `${current.t.toFixed(1)} s`;
      el.pos.textContent = `${current.x.toFixed(2)}, ${current.y.toFixed(2)}`;
      el.heading.textContent = `${current.heading.toFixed(1)} deg`;
      el.dist.textContent = `${current.dist.toFixed(2)} m`;
      el.err.textContent = `${current.err.toFixed(0)} deg`;
      el.motors.textContent = `${current.left} / ${current.right}`;
      el.state.textContent = current.state;
      scrub.value = String(Math.round(idx / Math.max(1, rows.length - 1) * 100));
    }

    function animate(ts) {
      if (!lastTs) lastTs = ts;
      if (playing && ts - lastTs > 45) {
        idx = Math.min(rows.length - 1, idx + 1);
        if (idx === rows.length - 1) {
          playing = false;
          play.textContent = 'Replay';
        }
        lastTs = ts;
        draw();
      }
      requestAnimationFrame(animate);
    }

    play.addEventListener('click', () => {
      if (idx === rows.length - 1) idx = 0;
      playing = !playing;
      play.textContent = playing ? 'Pause' : 'Play';
      draw();
    });
    scrub.addEventListener('input', () => {
      idx = Math.round(Number(scrub.value) / 100 * (rows.length - 1));
      playing = false;
      play.textContent = 'Play';
      draw();
    });

    const final = rows[rows.length - 1];
    const miss = Math.hypot(final.targetX - final.x, final.targetY - final.y);
    document.getElementById('summary').innerHTML =
      `<span>Target: ${final.targetX.toFixed(2)}, ${final.targetY.toFixed(2)} m</span>` +
      `<span>Final: ${final.x.toFixed(2)}, ${final.y.toFixed(2)} m</span>` +
      `<span>Miss: ${(miss * 100).toFixed(1)} cm</span>` +
      `<span>Steps: ${rows.length}</span>`;

    window.addEventListener('resize', resize);
    resize();
    requestAnimationFrame(animate);
  </script>
</body>
</html>
'@

$htmlContent = $template.Replace("__DATA__", $json)
Set-Content -Path $html -Value $htmlContent -Encoding UTF8
Write-Host "HTML viewer: $html"
Start-Process $html
