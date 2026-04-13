(function () {
  const config = window.BHL_WEBXR_CONFIG || {};
  const sceneEl = document.querySelector("a-scene");
  const enterXrButton = document.getElementById("enter-xr-button");
  const copyUrlButton = document.getElementById("copy-url-button");
  const wsStatusDot = document.getElementById("ws-status-dot");
  const wsStatusText = document.getElementById("ws-status-text");
  const xrStatusDot = document.getElementById("xr-status-dot");
  const xrStatusText = document.getElementById("xr-status-text");
  const controllerStatusDot = document.getElementById("controller-status-dot");
  const controllerStatusText = document.getElementById("controller-status-text");

  const sceneBannerEl = document.getElementById("scene-banner");
  const sceneFloorEl = document.getElementById("scene-floor");
  const leftControllerEl = document.getElementById("left-controller");
  const rightControllerEl = document.getElementById("right-controller");
  const leftControllerTextEl = document.getElementById("left-controller-text");
  const rightControllerTextEl = document.getElementById("right-controller-text");

  let websocket = null;
  let xrSessionActive = false;
  let leftGripActive = false;
  let rightGripActive = false;
  let leftTriggerActive = false;
  let rightTriggerActive = false;
  let leftControllerConnected = false;
  let rightControllerConnected = false;
  let leftGripOrigin = null;
  let rightGripOrigin = null;
  let lastSentAt = 0;
  let currentReferenceSpaceType = "local-floor";
  let preferLocalReferenceSpace = false;
  const sendIntervalMs = 34;
  const preferredReferenceSpaces = ["local-floor", "local"];
  const localFloorBannerPosition = "0 1.55 0";
  const localBannerPosition = "0 0 -0.9";
  const localFloorPlanePosition = "0 0 -0.1";
  const localPlanePosition = "0 -1.6 -0.1";

  const positionScale =
    Number.isFinite(Number(config.positionScale)) && Number(config.positionScale) > 0
      ? Number(config.positionScale)
      : 1.0;
  const webXrToRobotBasis = new THREE.Matrix3().set(
    0, 0, -1,
    -1, 0, 0,
    0, 1, 0
  );
  const leftHomePosition = new THREE.Vector3(0.2, 0.2, 0.6);
  const rightHomePosition = new THREE.Vector3(0.2, -0.2, 0.6);

  function updateReferenceSpaceVisuals(referenceSpaceType) {
    if (sceneBannerEl) {
      sceneBannerEl.setAttribute(
        "position",
        referenceSpaceType === "local-floor" ? localFloorBannerPosition : localBannerPosition
      );
    }
    if (sceneFloorEl) {
      sceneFloorEl.setAttribute(
        "position",
        referenceSpaceType === "local-floor" ? localFloorPlanePosition : localPlanePosition
      );
    }
  }

  function setStatus(dotEl, textEl, message, kind) {
    dotEl.classList.remove("connected", "warning");
    if (kind) {
      dotEl.classList.add(kind);
    }
    textEl.textContent = message;
  }

  function connectWebSocket() {
    if (!config.websocketUrl) {
      setStatus(wsStatusDot, wsStatusText, "Missing WebSocket URL", "warning");
      return;
    }

    websocket = new WebSocket(config.websocketUrl);
    websocket.addEventListener("open", () => {
      setStatus(wsStatusDot, wsStatusText, "WebSocket connected", "connected");
    });
    websocket.addEventListener("close", () => {
      setStatus(wsStatusDot, wsStatusText, "WebSocket disconnected", null);
      websocket = null;
      window.setTimeout(connectWebSocket, 1500);
    });
    websocket.addEventListener("error", () => {
      setStatus(wsStatusDot, wsStatusText, "WebSocket error", "warning");
    });
  }

  function syncReferenceSpaceType(referenceSpaceType) {
    currentReferenceSpaceType = referenceSpaceType;
    updateReferenceSpaceVisuals(referenceSpaceType);
    if (sceneEl) {
      sceneEl.setAttribute(
        "webxr",
        `referenceSpaceType: ${referenceSpaceType}; optionalFeatures: hand-tracking`
      );
    }
  }

  function referenceSpaceCandidates() {
    if (preferLocalReferenceSpace) {
      return ["local", "local-floor"];
    }
    return preferredReferenceSpaces.slice();
  }

  function configureControllerText(textEl) {
    if (!textEl) {
      return;
    }

    textEl.setAttribute("position", "0 0.045 -0.05");
    textEl.setAttribute("rotation", "-90 0 0");
    textEl.setAttribute("scale", "0.08 0.08 0.08");
    textEl.setAttribute("width", "1.9");
    textEl.setAttribute("align", "center");
    textEl.setAttribute("anchor", "center");
    textEl.setAttribute("baseline", "center");
    textEl.setAttribute("material", "side: double; transparent: true");
  }

  function syncControllerTrackingStatus(leftTracked, rightTracked) {
    const trackedCount = Number(leftTracked) + Number(rightTracked);
    if (trackedCount === 2) {
      setStatus(controllerStatusDot, controllerStatusText, "Both controllers tracked", "connected");
      return;
    }
    if (trackedCount === 1) {
      setStatus(controllerStatusDot, controllerStatusText, "One controller tracked", "warning");
      return;
    }
    setStatus(controllerStatusDot, controllerStatusText, "Waiting for controllers", null);
  }

  function setControllerConnection(hand, connected) {
    if (hand === "left") {
      leftControllerConnected = connected;
    } else {
      rightControllerConnected = connected;
    }
    syncControllerTrackingStatus(leftControllerConnected, rightControllerConnected);
  }

  function formatSigned(value, digits) {
    return `${value >= 0 ? "+" : ""}${value.toFixed(digits)}`;
  }

  function formatVector(vector, digits = 2) {
    return `${formatSigned(vector.x, digits)} ${formatSigned(vector.y, digits)} ${formatSigned(vector.z, digits)}`;
  }

  function worldToRobotVector(worldVector) {
    return worldVector.clone().applyMatrix3(webXrToRobotBasis);
  }

  function worldToRobotTarget(worldPosition, homePosition) {
    return homePosition.clone().add(worldToRobotVector(worldPosition).multiplyScalar(positionScale));
  }

  function quaternionToDegrees(quaternion) {
    const euler = new THREE.Euler().setFromQuaternion(quaternion, "YXZ");
    return new THREE.Vector3(
      THREE.MathUtils.radToDeg(euler.x),
      THREE.MathUtils.radToDeg(euler.y),
      THREE.MathUtils.radToDeg(euler.z)
    );
  }

  function ensureAxisIndicators(controllerEl, hand) {
    if (!controllerEl || controllerEl.dataset.axesReady === "true") {
      return;
    }

    const origin = document.createElement("a-sphere");
    origin.setAttribute("radius", "0.006");
    origin.setAttribute("color", "#f8fafc");
    controllerEl.appendChild(origin);

    function addAxis(prefix, color, position, rotation) {
      const cylinder = document.createElement("a-cylinder");
      cylinder.setAttribute("id", `${hand}-${prefix}-axis`);
      cylinder.setAttribute("height", "0.08");
      cylinder.setAttribute("radius", "0.003");
      cylinder.setAttribute("color", color);
      cylinder.setAttribute("position", position);
      cylinder.setAttribute("rotation", rotation);
      controllerEl.appendChild(cylinder);

      const cone = document.createElement("a-cone");
      cone.setAttribute("height", "0.015");
      cone.setAttribute("radius-bottom", "0.008");
      cone.setAttribute("radius-top", "0");
      cone.setAttribute("color", color);
      if (prefix === "x") {
        cone.setAttribute("position", "0.055 0 0");
        cone.setAttribute("rotation", "0 0 90");
      } else if (prefix === "y") {
        cone.setAttribute("position", "0 0.055 0");
      } else {
        cone.setAttribute("position", "0 0 0.055");
        cone.setAttribute("rotation", "90 0 0");
      }
      controllerEl.appendChild(cone);
    }

    addAxis("x", "#ff4d4f", "0.04 0 0", "0 0 90");
    addAxis("y", "#52c41a", "0 0.04 0", "0 0 0");
    addAxis("z", "#1677ff", "0 0 0.04", "90 0 0");
    controllerEl.dataset.axesReady = "true";
  }

  function updateControllerText(textEl, label, payload) {
    if (!textEl) {
      return;
    }

    if (!payload.connected) {
      textEl.setAttribute("value", `${label}\nNot tracked`);
      return;
    }

    const lines = [
      `${label}`,
      `W ${formatVector(payload.worldPosition)}`,
      `R ${formatVector(payload.robotPosition)}`,
      `D ${formatVector(payload.robotDelta)}`,
      `E ${formatVector(payload.eulerDegrees, 0)}`,
      `G ${payload.gripActive ? 1 : 0} T ${payload.trigger.toFixed(2)}`,
    ];
    textEl.setAttribute("value", lines.join("\n"));
  }

  function controllerPayload(controllerEl, hand, gripActive, triggerActive, gripOrigin) {
    const homePosition = hand === "left" ? leftHomePosition : rightHomePosition;
    const trackedControls = controllerEl?.components?.["tracked-controls"];
    const controllerVisible = Boolean(controllerEl?.object3D?.visible);
    const connected =
      Boolean(trackedControls && trackedControls.controller && controllerVisible) ||
      ((hand === "left" ? leftControllerConnected : rightControllerConnected) && controllerVisible);

    if (!controllerEl || !controllerEl.object3D || !connected) {
      return {
        hand,
        connected: false,
        gripActive,
        trigger: triggerActive ? 1 : 0,
        worldPosition: new THREE.Vector3(),
        robotPosition: homePosition.clone(),
        robotDelta: new THREE.Vector3(),
        eulerDegrees: new THREE.Vector3(),
        position: { x: 0, y: 0, z: 0 },
        quaternion: { x: 0, y: 0, z: 0, w: 1 },
      };
    }

    controllerEl.object3D.updateMatrixWorld(true);
    const worldPosition = new THREE.Vector3();
    const worldQuaternion = new THREE.Quaternion();
    const worldScale = new THREE.Vector3();
    controllerEl.object3D.matrixWorld.decompose(worldPosition, worldQuaternion, worldScale);
    const robotPosition = worldToRobotTarget(worldPosition, homePosition);
    const worldDelta = gripOrigin ? worldPosition.clone().sub(gripOrigin) : new THREE.Vector3();
    const robotDelta = worldToRobotVector(worldDelta).multiplyScalar(positionScale);
    const eulerDegrees = quaternionToDegrees(worldQuaternion);

    return {
      hand,
      connected,
      gripActive,
      trigger: triggerActive ? 1 : 0,
      worldPosition,
      robotPosition,
      robotDelta,
      eulerDegrees,
      position: {
        x: worldPosition.x,
        y: worldPosition.y,
        z: worldPosition.z,
      },
      quaternion: {
        x: worldQuaternion.x,
        y: worldQuaternion.y,
        z: worldQuaternion.z,
        w: worldQuaternion.w,
      },
    };
  }

  function sendControllerState(now) {
    const leftPayload = controllerPayload(
      leftControllerEl,
      "left",
      leftGripActive,
      leftTriggerActive,
      leftGripOrigin
    );
    const rightPayload = controllerPayload(
      rightControllerEl,
      "right",
      rightGripActive,
      rightTriggerActive,
      rightGripOrigin
    );

    updateControllerText(leftControllerTextEl, "Left", leftPayload);
    updateControllerText(rightControllerTextEl, "Right", rightPayload);
    syncControllerTrackingStatus(leftPayload.connected, rightPayload.connected);

    if (!websocket || websocket.readyState !== WebSocket.OPEN) {
      return;
    }
    if (now - lastSentAt < sendIntervalMs) {
      return;
    }
    lastSentAt = now;

    websocket.send(
      JSON.stringify({
        timestamp: Date.now(),
        leftController: leftPayload,
        rightController: rightPayload,
      })
    );
  }

  AFRAME.registerComponent("quest-teleop-bridge", {
    tick(time) {
      sendControllerState(time);
    },
  });

  function registerControllerEvents(controllerEl, hand) {
    if (!controllerEl) {
      return;
    }

    ensureAxisIndicators(controllerEl, hand);
    configureControllerText(hand === "left" ? leftControllerTextEl : rightControllerTextEl);

    controllerEl.addEventListener("controllerconnected", () => {
      setControllerConnection(hand, true);
    });
    controllerEl.addEventListener("controllerdisconnected", () => {
      setControllerConnection(hand, false);
    });

    controllerEl.addEventListener("gripdown", () => {
      const currentPosition = new THREE.Vector3();
      controllerEl.object3D.getWorldPosition(currentPosition);
      if (hand === "left") {
        leftGripActive = true;
        leftGripOrigin = currentPosition;
      } else {
        rightGripActive = true;
        rightGripOrigin = currentPosition;
      }
    });
    controllerEl.addEventListener("gripup", () => {
      if (hand === "left") {
        leftGripActive = false;
        leftGripOrigin = null;
      } else {
        rightGripActive = false;
        rightGripOrigin = null;
      }
    });
    controllerEl.addEventListener("triggerdown", () => {
      if (hand === "left") {
        leftTriggerActive = true;
      } else {
        rightTriggerActive = true;
      }
    });
    controllerEl.addEventListener("triggerup", () => {
      if (hand === "left") {
        leftTriggerActive = false;
      } else {
        rightTriggerActive = false;
      }
    });
  }

  function updateXrStatus(active) {
    xrSessionActive = active;
    setStatus(
      xrStatusDot,
      xrStatusText,
      active ? `XR active (${currentReferenceSpaceType})` : "XR session idle",
      active ? "connected" : null
    );
  }

  async function enterXrSession() {
    if (!sceneEl) {
      setStatus(xrStatusDot, xrStatusText, "A-Frame scene missing", "warning");
      return;
    }
    const failures = [];
    for (const referenceSpaceType of referenceSpaceCandidates()) {
      try {
        setStatus(
          xrStatusDot,
          xrStatusText,
          referenceSpaceType === "local-floor"
            ? "Starting XR with floor space"
            : "Retrying XR with local space",
          referenceSpaceType === "local-floor" ? null : "warning"
        );
        syncReferenceSpaceType(referenceSpaceType);
        await sceneEl.enterVR();
        preferLocalReferenceSpace = referenceSpaceType === "local";
        return;
      } catch (error) {
        console.error(error);
        failures.push(`${referenceSpaceType}: ${String(error)}`);
        if (referenceSpaceType === "local-floor") {
          preferLocalReferenceSpace = true;
        }
      }
    }
    setStatus(
      xrStatusDot,
      xrStatusText,
      `Failed to enter XR (${failures.join(" | ") || "unknown error"})`,
      "warning"
    );
  }

  async function copyPageUrl() {
    if (!config.pageUrl) {
      return;
    }
    try {
      await navigator.clipboard.writeText(config.pageUrl);
      copyUrlButton.textContent = "Copied";
      window.setTimeout(() => {
        copyUrlButton.textContent = "Copy Page URL";
      }, 1000);
    } catch (error) {
      console.error(error);
      copyUrlButton.textContent = "Copy failed";
      window.setTimeout(() => {
        copyUrlButton.textContent = "Copy Page URL";
      }, 1000);
    }
  }

  function bindSceneLifecycle() {
    if (!sceneEl) {
      return;
    }

    syncReferenceSpaceType(currentReferenceSpaceType);

    if (sceneEl.hasLoaded) {
      sceneEl.setAttribute("quest-teleop-bridge", "");
    } else {
      sceneEl.addEventListener("loaded", () => {
        sceneEl.setAttribute("quest-teleop-bridge", "");
      });
    }

    sceneEl.addEventListener("controllerconnected", () => {
      syncControllerTrackingStatus(leftControllerConnected, rightControllerConnected);
    });
    sceneEl.addEventListener("controllerdisconnected", () => {
      syncControllerTrackingStatus(leftControllerConnected, rightControllerConnected);
    });
    sceneEl.addEventListener("enter-vr", () => updateXrStatus(true));
    sceneEl.addEventListener("exit-vr", () => updateXrStatus(false));
  }

  function bootstrap() {
    configureControllerText(leftControllerTextEl);
    configureControllerText(rightControllerTextEl);
    registerControllerEvents(leftControllerEl, "left");
    registerControllerEvents(rightControllerEl, "right");
    bindSceneLifecycle();
    connectWebSocket();
    updateControllerText(leftControllerTextEl, "Left", { connected: false });
    updateControllerText(rightControllerTextEl, "Right", { connected: false });
    syncControllerTrackingStatus(false, false);

    if (enterXrButton) {
      enterXrButton.addEventListener("click", enterXrSession);
    }
    if (copyUrlButton) {
      copyUrlButton.addEventListener("click", copyPageUrl);
    }

    if (!window.isSecureContext) {
      setStatus(xrStatusDot, xrStatusText, "WebXR requires HTTPS", "warning");
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", bootstrap);
  } else {
    bootstrap();
  }
})();
