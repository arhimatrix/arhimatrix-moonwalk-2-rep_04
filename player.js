/* Moonwalk 2.0 · Report 04 — assembly player.
   Plays a planned build (pad or dome) with the real Dragon kinematics: the robot model and the joint
   angles come from the planner, so what you see is the job list the robot would execute.
   Plan coordinates are Z-up metres; the scene root is rotated so Z points up on screen. */
(function () {
  'use strict';
  var DRAGON = null, DRAGON_P = null;

  function b64(s, Type) { var bin = atob(s), n = bin.length, u8 = new Uint8Array(n); for (var i = 0; i < n; i++) u8[i] = bin.charCodeAt(i); return new Type(u8.buffer); }
  function m4(a) { var m = new THREE.Matrix4(); m.set(a[0][0], a[0][1], a[0][2], a[0][3], a[1][0], a[1][1], a[1][2], a[1][3], a[2][0], a[2][1], a[2][2], a[2][3], a[3][0], a[3][1], a[3][2], a[3][3]); return m; }
  function m12(a) { var m = new THREE.Matrix4(); m.set(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], 0, 0, 0, 1); return m; }
  function rotAxis(ax, ang) { return new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(ax[0], ax[1], ax[2]).normalize(), ang); }
  function C(h) { return new THREE.Color(h).convertSRGBToLinear(); }
  function robotT(x, y, yaw, z0) { var m = new THREE.Matrix4().makeRotationZ(yaw); m.setPosition(x, y, z0); return m; }

  function loadDragon() {
    if (!DRAGON_P) DRAGON_P = fetch('data/dragon_web.json').then(function (r) { return r.json(); }).then(function (d) {
      var links = {};
      Object.keys(d.links).forEach(function (n) {
        var l = d.links[n], e = { rest: m4(l.rest) };
        if (l.v) {
          var q = b64(l.v, Uint16Array), pos = new Float32Array(q.length);
          for (var i = 0; i < q.length; i++) { var c = i % 3; pos[i] = l.lo[c] + q[i] * l.sc[c]; }
          var idx = l.f32 ? b64(l.f, Uint32Array) : b64(l.f, Uint16Array);
          var g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(pos, 3)); g.setIndex(new THREE.BufferAttribute(idx, 1));
          g = g.toNonIndexed(); g.computeVertexNormals(); e.geo = g;
        }
        links[n] = e;
      });
      var J = d.chain.joints.map(function (j) { return { T: m4(j.T), post: m4(j.post), axis: j.axis, rev: j.type === 'revolute' || j.type === 'continuous' }; });
      var tip = new THREE.Matrix4().makeTranslation(d.chain.tip_offset[0], d.chain.tip_offset[1], d.chain.tip_offset[2]);
      var A = links[d.arm_base].rest, tipInv = links[d.tip].rest.clone().invert(), grip = {};
      d.grip_parts.forEach(function (g) { grip[g] = tipInv.clone().multiply(links[g].rest); });
      DRAGON = { links: links, J: J, children: d.chain_children, tipT: tip, A: A, tip: d.tip, grip: grip, tyres: d.tyres, z0: d.z0 };
      return DRAGON;
    });
    return DRAGON_P;
  }
  // forward kinematics: child frames in the arm-base frame + tool frame (port of moonwalk_gh.Chain.frames)
  function fk(D, q) {
    var T = new THREE.Matrix4(), k = 0, out = {};
    for (var i = 0; i < D.J.length; i++) {
      var j = D.J[i]; T.multiply(j.T);
      if (j.rev) { T.multiply(rotAxis(j.axis, q[k])); k++; }
      T.multiply(j.post);
      if (D.children[i]) out[D.children[i]] = T.clone();
    }
    return { frames: out, tool: T.clone().multiply(D.tipT) };
  }
  function pose(D, RT, q, tyre) {
    var out = {}, f = fk(D, q), Aw = RT.clone().multiply(D.A);
    Object.keys(D.links).forEach(function (n) { out[n] = RT.clone().multiply(D.links[n].rest); });
    Object.keys(f.frames).forEach(function (c) { out[c] = Aw.clone().multiply(f.frames[c]); });
    Object.keys(D.grip).forEach(function (g) { out[g] = out[D.tip].clone().multiply(D.grip[g]); });
    D.tyres.forEach(function (t) { var c = t.centre; var spin = new THREE.Matrix4().makeTranslation(c[0], c[1], c[2]).multiply(new THREE.Matrix4().makeRotationY(tyre)).multiply(new THREE.Matrix4().makeTranslation(-c[0], -c[1], -c[2]));
      out[t.name] = RT.clone().multiply(spin).multiply(D.links[t.name].rest); });
    return { links: out, tool: Aw.clone().multiply(f.tool) };
  }

  function Player(el, planUrl, opts) {
    opts = opts || {}; var self = this;
    this.el = el; this.opts = opts;
    var cv = el.querySelector('canvas'); this.cv = cv;
    var R = new THREE.WebGLRenderer({ canvas: cv, antialias: true, preserveDrawingBuffer: !!opts.rec }); R.setPixelRatio(opts.rec ? 1 : Math.min(devicePixelRatio || 1, 2));
    R.outputEncoding = THREE.sRGBEncoding; R.toneMapping = THREE.ACESFilmicToneMapping; R.toneMappingExposure = 1.05;
    R.shadowMap.enabled = !opts.noShadow; R.shadowMap.type = THREE.PCFSoftShadowMap; this.R = R;
    var sc = new THREE.Scene(); sc.background = C(0x0B0F16); sc.fog = new THREE.Fog(C(0x0B0F16), 16, 34); this.sc = sc;
    var cam = new THREE.PerspectiveCamera(32, 1, 0.05, 200); this.cam = cam;
    var root = new THREE.Group(); root.rotation.x = -Math.PI / 2; sc.add(root); this.root = root;      // plan Z-up -> screen Y-up
    sc.add(new THREE.HemisphereLight(0xc7d3ff, 0x2a2622, 0.55));
    var sun = new THREE.DirectionalLight(0xfff3e2, 1.6); sun.position.set(-6, 5, 7); sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.left = -7; sun.shadow.camera.right = 7; sun.shadow.camera.top = 7; sun.shadow.camera.bottom = -7; sun.shadow.bias = -0.0005; sc.add(sun); this.sun = sun;
    var rim = new THREE.DirectionalLight(0x9fb4ff, 0.5); rim.position.set(6, 3, -5); sc.add(rim);
    this.t = 0; this.ghosts = opts.ghosts !== false; this.playing = !opts.rec; this.mode = opts.mode || 'detail'; this.az = opts.az != null ? opts.az : -0.9; this.el0 = 0.48; this.zoom = 1; this.orbit = true;
    var hud = el.querySelector('.hud'); this.hud = hud;
    Promise.all([loadDragon(), fetch(planUrl).then(function (r) { return r.json(); }), opts.plinth ? fetch(opts.plinth).then(function (r) { return r.json(); }) : null])
      .then(function (a) { self.build(a[0], a[1], a[2]); });
  }
  Player.prototype.build = function (D, P, plinth) {
    var self = this, root = this.root; this.D = D; this.P = P; var M = P.meta, gz = M.ground;
    // ground + markings
    var ground = new THREE.Mesh(new THREE.CircleGeometry(16, 96), new THREE.MeshStandardMaterial({ color: C(0x3b3a38), roughness: 1 }));
    ground.position.set(M.c0[0], M.c0[1], gz - 0.002); ground.receiveShadow = true; root.add(ground);
    var lineM = new THREE.LineBasicMaterial({ color: C(0x5b6475), transparent: true, opacity: 0.6 });
    if (M.kind === 'pad') { var ring = [], rr = M.ring_r; for (var i = 0; i <= 128; i++) { var a = i / 128 * Math.PI * 2; ring.push(new THREE.Vector3(M.c0[0] + rr * Math.cos(a), M.c0[1] + rr * Math.sin(a), gz + 0.003)); }
      root.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(ring), lineM)); }
    // stock yard pads
    M.stacks.forEach(function (s) { var m = new THREE.Mesh(new THREE.CircleGeometry(0.2, 6), new THREE.MeshStandardMaterial({ color: C(0x55524d), roughness: 1 })); m.position.set(s[0], s[1], gz + 0.001); root.add(m); });
    // plinth (dome)
    var plM = new THREE.MeshStandardMaterial({ color: C(0x8a8378), roughness: 0.95, flatShading: true });
    if (plinth) plinth.forEach(function (b) { root.add(meshOf(b, plM, true)); });
    // blocks
    var bM = new THREE.MeshStandardMaterial({ color: C(0xc9b99a), roughness: 0.92, flatShading: true });
    var gM = new THREE.MeshStandardMaterial({ color: C(0xd9b596), roughness: 1, transparent: true, opacity: M.kind === 'dome' ? 0.05 : 0.12, depthWrite: false });
    var edgeM = new THREE.LineBasicMaterial({ color: C(0x2b2722), transparent: true, opacity: 0.35 });
    this.blocks = P.blocks.map(function (b) {
      var B = m12(b.B), g = new THREE.Group(); g.matrixAutoUpdate = false;
      var mesh = meshOf(b.mesh, bM, false); mesh.castShadow = true; mesh.receiveShadow = true; g.add(mesh);
      g.add(new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 25), edgeM));
      var ghost = meshOf(b.mesh, gM, false); ghost.visible = false; root.add(ghost);
      root.add(g); return { g: g, Binv: B.clone().invert(), ghost: ghost, info: b };
    });
    this.planned = P.planned.map(function (p) { return { idx: p.idx, stock: m12(p.stock), G: m12(p.G), ta: p.t_appear, tp: p.t_pick, tl: p.t_place }; });
    this.plannedOf = {}; this.planned.forEach(function (p, i) { self.plannedOf[p.idx] = i; });
    // robot
    var rM = new THREE.MeshStandardMaterial({ color: C(0xe9e7e2), roughness: 0.45, metalness: 0.1 }), aM = new THREE.MeshStandardMaterial({ color: C(0xe88a6f), roughness: 0.5, metalness: 0.1 });
    this.rlinks = {};
    Object.keys(D.links).forEach(function (n) { var l = D.links[n]; if (!l.geo) return; var m = new THREE.Mesh(l.geo, /leg201/.test(n) ? aM : rM); m.matrixAutoUpdate = false; m.castShadow = true; root.add(m); self.rlinks[n] = m; });
    this.S = P.samples; this.tTot = M.t_total; this.tDetail = M.detail_end;
    // camera target
    var ext = M.kind === 'dome' ? M.R_base + 1.9 : M.ring_r * 0.85; this.ext = ext;
    this.center = new THREE.Vector3(M.c0[0] - 0.5, gz + (M.kind === 'dome' ? 0.6 : 0.2), -M.c0[1]);
    this.ready = true; this.setT(0); this.size();
    addEventListener('resize', function () { self.size(); });
    this.bindUI();
    if (!this.opts.rec) requestAnimationFrame(function f(now) { self.tick(now); requestAnimationFrame(f); });
    if (this.opts.onReady) this.opts.onReady(this);
  };
  function meshOf(b, mat, shadow) {
    var g = new THREE.BufferGeometry(), v = b.v, idx = [];
    b.f.forEach(function (f) { for (var j = 1; j < f.length - 1; j++) idx.push(f[0], f[j], f[j + 1]); });
    g.setAttribute('position', new THREE.Float32BufferAttribute(v, 3)); g.setIndex(idx); g = g.toNonIndexed(); g.computeVertexNormals();
    var m = new THREE.Mesh(g, mat); if (shadow) { m.castShadow = true; m.receiveShadow = true; } return m;
  }
  Player.prototype.sample = function (t) {
    var S = this.S, lo = 0, hi = S.length - 1;
    if (t <= S[0][0]) return S[0]; if (t >= S[hi][0]) return S[hi];
    while (hi - lo > 1) { var mid = (lo + hi) >> 1; if (S[mid][0] <= t) lo = mid; else hi = mid; }
    var a = S[lo], b = S[hi], u = (t - a[0]) / Math.max(1e-6, b[0] - a[0]), o = [t];
    for (var i = 1; i < a.length; i++) { var x = a[i], y = b[i]; if (i === 3) { var d = y - x; while (d > Math.PI) d -= 2 * Math.PI; while (d < -Math.PI) d += 2 * Math.PI; o.push(x + d * u); } else o.push(x + (y - x) * u); }
    return o;
  };
  Player.prototype.setT = function (t) {
    if (!this.ready) return; this.t = t; var D = this.D, s = this.sample(t);
    var RT = robotT(s[1], s[2], s[3], this.P.meta.ground + D.z0), q = s.slice(5, 12), P = pose(D, RT, q, s[4]), self = this;
    Object.keys(this.rlinks).forEach(function (n) { var m = self.rlinks[n]; m.matrix.copy(P.links[n]); m.matrixWorldNeedsUpdate = true; });
    var placed = 0, ghosts = this.ghosts;
    this.blocks.forEach(function (b, idx) {
      var pi = self.plannedOf[idx], X = null;
      if (pi == null) { b.g.visible = false; b.ghost.visible = !!ghosts && b.info.zone !== 'doorway'; return; }
      var p = self.planned[pi];
      if (t < p.ta) { b.g.visible = false; }
      else if (t < p.tp) { X = p.stock.clone().multiply(b.Binv); }
      else if (t < p.tl) { X = P.tool.clone().multiply(p.G).multiply(b.Binv); }
      else { X = new THREE.Matrix4(); placed++; }
      if (X) { b.g.visible = true; b.g.matrix.copy(X); b.g.matrixWorldNeedsUpdate = true; }
      b.ghost.visible = !!ghosts && t < p.tl;
    });
    this.placed = placed;
    if (this.hud) {
      var A = this.P.acts, act = A[0][1]; for (var i = 0; i < A.length && A[i][0] <= t; i++) act = A[i][1];
      var h = Math.floor(t / 3600), mnt = Math.floor(t % 3600 / 60), sec = Math.floor(t % 60);
      this.hud.querySelector('.act').textContent = act.replace(/^block (\d+)\/(\d+):/, 'Step $1 of $2 ·');
      this.hud.querySelector('.cnt').textContent = placed + ' / ' + this.planned.length + ' blocks set';
      this.hud.querySelector('.clk').textContent = 'robot time ' + (h ? h + ' h ' : '') + (mnt < 10 && h ? '0' : '') + mnt + ' min ' + (sec < 10 ? '0' : '') + sec + ' s';
      var sl = this.el.querySelector('input[type=range]'); if (sl && !this.dragging) sl.value = (t / this.span()) * 1000;
    }
  };
  Player.prototype.span = function () { return this.mode === 'detail' ? this.tDetail : this.tTot; };
  Player.prototype.rate = function () { return this.mode === 'detail' ? 6 : this.tTot / 120; };     // detail at 6x, whole build in 2 min
  Player.prototype.camera = function (az) {
    var c = this.center, r = this.ext * 2.05 * this.zoom, el = this.el0;
    this.cam.position.set(c.x + r * Math.cos(el) * Math.cos(az), c.y + r * Math.sin(el), c.z + r * Math.cos(el) * Math.sin(az)); this.cam.lookAt(c);
  };
  Player.prototype.render = function () { this.camera(this.az); this.R.render(this.sc, this.cam); };
  Player.prototype.tick = function (now) {
    var dt = this.last ? Math.min(0.1, (now - this.last) / 1000) : 0; this.last = now;
    var r = this.cv.getBoundingClientRect(); if (r.bottom < 0 || r.top > innerHeight) return;
    if (this.playing) { var t = this.t + dt * this.rate(); if (t > this.span()) t = 0; this.setT(t); }
    if (this.orbit && this.playing) this.az += dt * 0.06;
    this.render();
  };
  Player.prototype.size = function () { var w = this.cv.clientWidth, h = this.cv.clientHeight; this.R.setSize(w, h, false); this.cam.aspect = w / h; this.cam.updateProjectionMatrix(); };
  Player.prototype.bindUI = function () {
    var self = this, el = this.el;
    var play = el.querySelector('.play'), sl = el.querySelector('input[type=range]'), modes = el.querySelectorAll('[data-mode]'), gh = el.querySelector('.ghost');
    if (play) play.addEventListener('click', function () { self.playing = !self.playing; play.textContent = self.playing ? 'Pause' : 'Play'; play.setAttribute('aria-pressed', self.playing); });
    if (sl) { sl.addEventListener('input', function () { self.dragging = true; self.setT(sl.value / 1000 * self.span()); }); sl.addEventListener('change', function () { self.dragging = false; }); }
    modes.forEach(function (b) { b.addEventListener('click', function () { self.mode = b.dataset.mode; modes.forEach(function (x) { x.setAttribute('aria-pressed', x === b); }); self.setT(0); }); });
    if (gh) gh.addEventListener('click', function () { self.ghosts = !self.ghosts; gh.setAttribute('aria-pressed', self.ghosts); self.setT(self.t); });
    var drag = null; this.cv.addEventListener('pointerdown', function (e) { drag = [e.clientX, e.clientY, self.az, self.el0]; self.cv.setPointerCapture(e.pointerId); });
    this.cv.addEventListener('pointermove', function (e) { if (!drag) return; self.az = drag[2] + (e.clientX - drag[0]) * 0.006; self.el0 = Math.max(0.08, Math.min(1.35, drag[3] + (e.clientY - drag[1]) * 0.005)); });
    this.cv.addEventListener('pointerup', function () { drag = null; });
    this.cv.addEventListener('wheel', function (e) { if (!e.ctrlKey && !e.metaKey) return; e.preventDefault(); self.zoom = Math.max(0.35, Math.min(1.8, self.zoom * (e.deltaY > 0 ? 1.08 : 0.93))); }, { passive: false });
  };
  window.MoonwalkPlayer = Player;
})();
