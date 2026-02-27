"use strict";

const path = require("path");
let express;

const KTS_TO_MS = 0.514444;
const DEG_TO_RAD = Math.PI / 180;
const TWO_PI    = 2 * Math.PI;

function wrapRad(r) { return ((r % TWO_PI) + TWO_PI) % TWO_PI; }

// ── Server-side simulation ────────────────────────────────────────────────────

const DEFAULT_PARAMS = {
  tideDir: 45,   tideSpeed: 1.5,  tideDirSig: 0,    tideSpeedSig: 0,
  windDir: 225,  windSpeed: 12,   windDirSig: 0,    windSpeedSig: 0,
  boatHdg: 45,   boatSTW: 6.5,   boatHdgSig: 0,    boatSTWSig: 0,
  yawAmp: 0,     yawPeriod: 8,   yawSig: 0,
  pitchAmp: 0,   pitchPeriod: 5, pitchSig: 0,
  rollMean: 0,   rollAmp: 0,     rollPeriod: 4,    rollSig: 0,
  mastHeight: 18, variation: 0
};

function randn() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function computeSample(p, t) {
  const tws     = Math.max(0, p.windSpeed  + p.windSpeedSig  * randn());
  const twd     = (p.windDir   + p.windDirSig   * randn() + 360) % 360;
  const hdgMag  = (p.boatHdg   + p.boatHdgSig   * randn() + 360) % 360;
  const stw     = Math.max(0, p.boatSTW    + p.boatSTWSig    * randn());
  const hdgTrue = (hdgMag + p.variation + 360) % 360;
  const tideSpd = Math.max(0, p.tideSpeed  + p.tideSpeedSig  * randn());
  const tideDir = (p.tideDir   + p.tideDirSig   * randn() + 360) % 360;

  const roll    = p.rollMean  + p.rollAmp  * Math.sin(2 * Math.PI * t / p.rollPeriod)         + p.rollSig  * randn();
  const pitch   =               p.pitchAmp * Math.sin(2 * Math.PI * t / p.pitchPeriod + 0.7)  + p.pitchSig * randn();
  const yawRate =               p.yawAmp   * Math.sin(2 * Math.PI * t / p.yawPeriod   + 1.2)  + p.yawSig   * randn();

  const mH          = p.mastHeight;
  const rollRad     = roll * DEG_TO_RAD;
  const pitchRateRad = (p.pitchAmp * 2 * Math.PI / p.pitchPeriod) * Math.cos(2 * Math.PI * t / p.pitchPeriod + 0.7) * DEG_TO_RAD;
  const rollRateRad  = (p.rollAmp  * 2 * Math.PI / p.rollPeriod)  * Math.cos(2 * Math.PI * t / p.rollPeriod)        * DEG_TO_RAD;
  const mastVx = -pitchRateRad * mH * Math.cos(rollRad);
  const mastVy =  rollRateRad  * mH;

  const twdRad  = twd      * DEG_TO_RAD;
  const hdgRad  = hdgTrue  * DEG_TO_RAD;
  const twVx = -tws * Math.sin(twdRad);
  const twVy = -tws * Math.cos(twdRad);
  const bVx  =  stw * Math.sin(hdgRad);
  const bVy  =  stw * Math.cos(hdgRad);

  const awVx = twVx - bVx;
  const awVy = twVy - bVy;
  const aws  = Math.sqrt(awVx * awVx + awVy * awVy);

  // atan2(awVx,awVy) gives direction the wind vector is going TO; +180 converts to FROM
  const awFromDeg = (Math.atan2(awVx, awVy) / DEG_TO_RAD + 180 + 360) % 360;
  let awaRel = awFromDeg - hdgTrue;
  while (awaRel >  180) awaRel -= 360;
  while (awaRel < -180) awaRel += 360;

  const sinH = Math.sin(hdgRad), cosH = Math.cos(hdgRad);
  const mastVxGlobal = mastVx * sinH + mastVy * cosH;
  const mastVyGlobal = mastVx * cosH - mastVy * sinH; // eslint-disable-line no-unused-vars

  const tideDirRad = tideDir * DEG_TO_RAD;
  const tiVx = tideSpd * Math.sin(tideDirRad);
  const tiVy = tideSpd * Math.cos(tideDirRad);
  const sogVx = bVx + tiVx;
  const sogVy = bVy + tiVy;
  const sog = Math.sqrt(sogVx * sogVx + sogVy * sogVy);
  const cog = (Math.atan2(sogVx, sogVy) / DEG_TO_RAD + 360) % 360;

  const twaDeg = ((twd - hdgTrue) % 360 + 360) % 360;
  const twa = twaDeg > 180 ? twaDeg - 360 : twaDeg;

  // awsRaw: apparent wind at masthead sensor including mast motion
  const awRawVx = awVx + mastVxGlobal;
  const awRawVy = awVy + (mastVx * cosH - mastVy * sinH);
  const awsRaw  = Math.sqrt(awRawVx * awRawVx + awRawVy * awRawVy); // eslint-disable-line no-unused-vars

  return { tws, twd, hdg: hdgMag, stw, sog, cog, tideDir, tideSpd,
           aws, awa: awaRel, twa, roll, pitch, yawRate };
}

function buildSkValues(s, variationDeg) {
  const hdgMag  = wrapRad(s.hdg * DEG_TO_RAD);
  const hdgTrue = wrapRad(hdgMag + variationDeg * DEG_TO_RAD);
  return [
    { path: 'environment.wind.speedApparent',    value: s.aws      * KTS_TO_MS },
    { path: 'environment.wind.angleApparent',    value: s.awa      * DEG_TO_RAD },
    { path: 'environment.wind.speedTrue',        value: s.tws      * KTS_TO_MS },
    { path: 'environment.wind.angleTrueWater',   value: s.twa      * DEG_TO_RAD },
    { path: 'environment.wind.directionTrue',    value: wrapRad(s.twd * DEG_TO_RAD) },
    { path: 'navigation.headingMagnetic',        value: hdgMag },
    { path: 'navigation.headingTrue',            value: hdgTrue },
    { path: 'navigation.speedThroughWater',      value: s.stw      * KTS_TO_MS },
    { path: 'navigation.speedOverGround',        value: s.sog      * KTS_TO_MS },
    { path: 'navigation.courseOverGroundTrue',   value: wrapRad(s.cog * DEG_TO_RAD) },
    { path: 'environment.current.setTrue',       value: wrapRad(s.tideDir * DEG_TO_RAD) },
    { path: 'environment.current.drift',         value: s.tideSpd  * KTS_TO_MS },
    { path: 'navigation.attitude',               value: { roll: s.roll * DEG_TO_RAD, pitch: s.pitch * DEG_TO_RAD, yaw: hdgTrue } },
    { path: 'navigation.rateOfTurn',             value: s.yawRate  * DEG_TO_RAD },
  ];
}

// ─────────────────────────────────────────────────────────────────────────────

module.exports = function (app) {
  const plugin = {};

  plugin.id = "sailsim-plugin";
  plugin.name = "Sailing Simulator";
  plugin.description =
    "Generates realistic sailing sensor data (wind, heading, speed, attitude) for testing downstream consumers";

  // Server-side sim state (persists across browser sessions)
  let simParams = { ...DEFAULT_PARAMS };
  let simState  = { running: false, time: 0, rate: 200 };
  let simTimer  = null;

  // --- Configuration schema ---
  plugin.schema = {
    type: "object",
    properties: {
      mastHeight: {
        type: "number",
        title: "Mast sensor height (m)",
        description: "Height of the wind sensor above the waterline",
        default: 18,
      },
      variationFallback: {
        type: "number",
        title: "Magnetic variation (degrees)",
        description: "Initial magnetic variation. Positive = East.",
        default: 0,
      },
    },
  };

  let options = {};

  function serverSimStep() {
    simState.time += simState.rate / 1000;
    const s = computeSample(simParams, simState.time);
    const values = buildSkValues(s, simParams.variation);
    values.push({ path: 'navigation.magneticVariation', value: simParams.variation * DEG_TO_RAD });
    try {
      app.handleMessage(plugin.id, {
        updates: [{ source: { label: plugin.id }, timestamp: new Date().toISOString(), values }]
      });
    } catch (e) {
      app.error('sailsim: handleMessage failed: ' + e.message);
    }
  }

  function startServerSim(rateMs) {
    if (simTimer) clearInterval(simTimer);
    simState.rate    = rateMs || simState.rate;
    simState.running = true;
    simTimer = setInterval(serverSimStep, simState.rate);
  }

  function stopServerSim() {
    if (simTimer) { clearInterval(simTimer); simTimer = null; }
    simState.running = false;
  }

  // --- Lifecycle ---
  plugin.start = function (opts) {
    options = opts || {};
    if (options.mastHeight    == null) options.mastHeight    = 18;
    if (options.variationFallback == null) options.variationFallback = 0;
    if (!options.variationPath)  options.variationPath  = "navigation.magneticVariation";

    // Seed sim params from plugin options
    simParams.mastHeight = options.mastHeight;
    simParams.variation  = options.variationFallback;

    startServerSim();
    app.debug("Sailing Simulator plugin started");
  };

  plugin.stop = function () {
    stopServerSim();
    app.debug("Sailing Simulator plugin stopped");
  };

  // --- Webapp routes ---
  plugin.registerWithRouter = function (router) {
    if (!express) express = require("express");

    const staticDir = path.join(__dirname, "ui");
    router.use(express.static(staticDir));
    router.use(express.json());

    if (app.use) {
      app.use("/sailsim-plugin", express.static(staticDir));
      app.use("/sailsim-plugin", express.json());
      app.get("/sailsim-plugin", (req, res) => {
        res.sendFile(path.join(staticDir, "index.html"));
      });
    }

    // GET /config — plugin settings for the UI
    function configHandler(req, res) {
      res.json({
        mastHeight:   options.mastHeight,
        variationDeg: simParams.variation,
      });
    }

    // GET /simstate — current server-side sim state + params (for UI sync on load)
    function simStateHandler(_req, res) {
      res.json({ ...simState, params: simParams });
    }

    // PUT /params — browser sends current slider/compass values; server updates sim params
    function paramsHandler(req, res) {
      const p = req.body;
      if (!p || typeof p !== 'object') return res.status(400).json({ error: 'invalid payload' });
      Object.assign(simParams, p);
      res.json({ ok: true });
    }

    // POST /control — start / stop / reset server sim loop
    function controlHandler(req, res) {
      const { action, rate } = req.body || {};
      if (action === 'start') {
        startServerSim(rate || simState.rate);
        res.json({ ok: true, running: true });
      } else if (action === 'stop') {
        stopServerSim();
        res.json({ ok: true, running: false });
      } else if (action === 'reset') {
        stopServerSim();
        simState.time = 0;
        res.json({ ok: true, running: false });
      } else {
        res.status(400).json({ error: 'unknown action' });
      }
    }

    // POST /data — legacy: browser-computed sample (kept for backward compat)
    function dataHandler(req, res) {
      const d = req.body;
      if (!d || typeof d !== "object") return res.status(400).json({ error: "invalid payload" });

      const variation   = d.variation != null ? d.variation * DEG_TO_RAD : 0;
      const hdgMag      = (d.hdg || 0) * DEG_TO_RAD;
      const hdgTrue     = hdgMag + variation;

      const values = [
        { path: "environment.wind.speedApparent",  value: (d.aws || 0) * KTS_TO_MS },
        { path: "environment.wind.angleApparent",  value: (d.awa || 0) * DEG_TO_RAD },
        { path: "environment.wind.speedTrue",      value: (d.tws || 0) * KTS_TO_MS },
        { path: "environment.wind.angleTrueWater", value: (d.twa || 0) * DEG_TO_RAD },
        { path: "environment.wind.directionTrue",  value: (d.twd || 0) * DEG_TO_RAD },
        { path: "navigation.headingMagnetic",      value: hdgMag },
        { path: "navigation.headingTrue",          value: hdgTrue },
        { path: "navigation.speedThroughWater",    value: (d.stw || 0) * KTS_TO_MS },
        { path: "navigation.speedOverGround",      value: (d.sog || 0) * KTS_TO_MS },
        { path: "navigation.courseOverGroundTrue", value: (d.cog || 0) * DEG_TO_RAD },
        { path: "environment.current.setTrue",     value: (d.tideDir || 0) * DEG_TO_RAD },
        { path: "environment.current.drift",       value: (d.tideSpd || 0) * KTS_TO_MS },
        { path: "navigation.attitude",             value: { roll: (d.roll || 0) * DEG_TO_RAD, pitch: (d.pitch || 0) * DEG_TO_RAD, yaw: hdgTrue } },
        { path: "navigation.rateOfTurn",           value: (d.yawRate || 0) * DEG_TO_RAD },
      ];
      if (d.variation != null) {
        values.push({ path: "navigation.magneticVariation", value: d.variation * DEG_TO_RAD });
      }

      app.handleMessage(plugin.id, { updates: [{ source: { label: plugin.id }, timestamp: new Date().toISOString(), values }] });
      res.json({ ok: true });
    }

    // Register on plugin router
    router.get("/config",   configHandler);
    router.get("/simstate", simStateHandler);
    router.put("/params",   paramsHandler);
    router.post("/control", controlHandler);
    router.post("/data",    dataHandler);
    router.get("/", (req, res) => res.sendFile(path.join(staticDir, "index.html")));

    // Register same handlers on main app for /sailsim-plugin/ webapp access
    if (app.get && app.post) {
      app.get("/sailsim-plugin/config",   configHandler);
      app.get("/sailsim-plugin/simstate", simStateHandler);
      app.put("/sailsim-plugin/params",   paramsHandler);
      app.post("/sailsim-plugin/control", controlHandler);
      app.post("/sailsim-plugin/data",    dataHandler);
    }
  };

  return plugin;
};
