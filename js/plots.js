// Lightweight time-series plotter on a canvas. Each Plot keeps a fixed-size
// circular buffer of (t, [series...]) samples and renders the most recent
// window. No external dependencies.

const COLORS = ["#4cc2ff", "#7ee787", "#f7b955", "#ff6b6b", "#a472ff", "#62d2a2", "#e8bd6f"];

export class Plot {
  constructor({ title, series, capacity = 4000 }) {
    this.title = title;
    this.series = series.map((s, i) => ({
      label: s.label,
      color: s.color || COLORS[i % COLORS.length],
    }));
    this.capacity = capacity;
    this.t = new Float32Array(capacity);
    this.data = this.series.map(() => new Float32Array(capacity));
    this.head = 0;
    this.size = 0;

    this.root = document.createElement("div");
    this.root.className = "plot";
    const head = document.createElement("div");
    head.className = "plot-title";
    const titleSpan = document.createElement("span");
    titleSpan.textContent = title;
    const legend = document.createElement("span");
    legend.className = "legend";
    for (const s of this.series) {
      const sp = document.createElement("span");
      const i = document.createElement("i");
      i.style.background = s.color;
      sp.appendChild(i);
      sp.appendChild(document.createTextNode(s.label));
      legend.appendChild(sp);
    }
    head.appendChild(titleSpan);
    head.appendChild(legend);
    this.root.appendChild(head);
    this.canvas = document.createElement("canvas");
    this.root.appendChild(this.canvas);
    this.ctx = this.canvas.getContext("2d");
    this.frozen = false;
    this.window = 10;
  }

  setFrozen(f) { this.frozen = f; }
  setWindow(s) { this.window = s; }

  push(t, values) {
    if (this.frozen) return;
    this.t[this.head] = t;
    for (let i = 0; i < values.length; i++) this.data[i][this.head] = values[i];
    this.head = (this.head + 1) % this.capacity;
    if (this.size < this.capacity) this.size++;
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth || 200;
    const h = this.canvas.clientHeight || 80;
    this.canvas.width = Math.floor(w * dpr);
    this.canvas.height = Math.floor(h * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this._cssW = w; this._cssH = h;
  }

  render(currentT) {
    this.resize();
    const ctx = this.ctx;
    const w = this._cssW, h = this._cssH;
    ctx.clearRect(0, 0, w, h);

    if (this.size === 0) return;

    const tEnd = currentT;
    const tStart = currentT - this.window;

    // Find min/max over visible window.
    let yMin = Infinity, yMax = -Infinity;
    let n = 0;
    for (let k = 0; k < this.size; k++) {
      const idx = (this.head - 1 - k + this.capacity) % this.capacity;
      if (this.t[idx] < tStart) break;
      n++;
      for (let s = 0; s < this.series.length; s++) {
        const v = this.data[s][idx];
        if (v < yMin) yMin = v;
        if (v > yMax) yMax = v;
      }
    }
    if (!isFinite(yMin) || !isFinite(yMax)) return;
    if (yMin === yMax) { yMin -= 1; yMax += 1; }
    const pad = (yMax - yMin) * 0.1;
    yMin -= pad; yMax += pad;

    // Background grid.
    ctx.strokeStyle = "rgba(255,255,255,0.05)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let i = 1; i < 4; i++) {
      const y = (i / 4) * h;
      ctx.moveTo(0, y); ctx.lineTo(w, y);
    }
    ctx.stroke();

    // Zero line if visible.
    if (yMin < 0 && yMax > 0) {
      const zy = h - ((0 - yMin) / (yMax - yMin)) * h;
      ctx.strokeStyle = "rgba(255,255,255,0.2)";
      ctx.beginPath();
      ctx.moveTo(0, zy); ctx.lineTo(w, zy);
      ctx.stroke();
    }

    // Axis labels (min/max).
    ctx.fillStyle = "rgba(255,255,255,0.4)";
    ctx.font = "9px ui-monospace, Menlo, monospace";
    ctx.textBaseline = "top";
    ctx.fillText(yMax.toFixed(2), 2, 1);
    ctx.textBaseline = "bottom";
    ctx.fillText(yMin.toFixed(2), 2, h - 1);

    // Plot each series.
    for (let s = 0; s < this.series.length; s++) {
      ctx.strokeStyle = this.series[s].color;
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      let drew = false;
      for (let k = n - 1; k >= 0; k--) {
        const idx = (this.head - 1 - k + this.capacity) % this.capacity;
        const t = this.t[idx];
        const v = this.data[s][idx];
        const x = ((t - tStart) / (tEnd - tStart)) * w;
        const y = h - ((v - yMin) / (yMax - yMin)) * h;
        if (!drew) { ctx.moveTo(x, y); drew = true; }
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  }

  clear() {
    this.head = 0;
    this.size = 0;
  }
}

export class PlotRack {
  constructor(container) {
    this.container = container;
    this.plots = [];
  }
  add(plot) {
    this.plots.push(plot);
    this.container.appendChild(plot.root);
  }
  setFrozen(f) { for (const p of this.plots) p.setFrozen(f); }
  setWindow(s) { for (const p of this.plots) p.setWindow(s); }
  render(t) { for (const p of this.plots) p.render(t); }
  clear() { for (const p of this.plots) p.clear(); }
}
