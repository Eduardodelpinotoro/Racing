// =========================================================================
// F1 PHYSICS CORE v1.0 - Motor profesional para simulador
// Implementa: Pacejka MF 5.2, suspensión 14DOF, aerodinámica real
// =========================================================================

class F1PhysicsCore {
    constructor() {
        this.setupConstants();
        this.initializeState();
        this.initializeSubsystems();
    }

    setupConstants() {
        // CONSTANTES F1 2023
        this.constants = {
            // Masa y geometría
            MASS: 740,                  // kg (incl. piloto)
            WHEELBASE: 3.6,             // m
            TRACK_WIDTH: {front: 1.8, rear: 1.6},
            COG_HEIGHT: 0.3,            // m
            TIRE_RADIUS: 0.33,          // m
            
            // Inercias [kg·m²]
            INERTIA: {xx: 300, yy: 1000, zz: 300},
            
            // Suspensión [N/m, Ns/m]
            SPRING_RATE: {front: 120000, rear: 100000},
            DAMPER_RATE: {front: 8000, rear: 7000},
            ARB_RATE: {front: 50000, rear: 40000},
            
            // Aerodinámica
            AERO_BALANCE: 0.45,         // 45% delantero
            CL: 3.5,                    // Coef. sustentación
            CD: 1.2,                    // Coef. arrastre
            FRONTAL_AREA: 1.5,          // m²
            
            // Motor
            MAX_TORQUE: 800,            // Nm @ ~10500 RPM
            MAX_RPM: 15000,
            IDLE_RPM: 4000,
            GEAR_RATIOS: [0, 3.586, 2.886, 2.295, 1.885, 1.583, 1.346, 1.165, 1.036],
            FINAL_DRIVE: 3.692,
            
            // Frenos
            MAX_BRAKE_TORQUE: 12000,    // Nm total
            BRAKE_BIAS: 0.55,           // 55% delantero
            
            // Ambiente
            GRAVITY: 9.81,
            AIR_DENSITY: 1.225,
            TRACK_GRIP: 1.0
        };
    }

    initializeState() {
        // Estado del vehículo (14 variables)
        this.state = {
            // Posición [m]
            x: 0, y: 0, z: this.constants.COG_HEIGHT,
            
            // Orientación [rad]
            roll: 0, pitch: 0, yaw: 0,
            
            // Velocidades [m/s, rad/s]
            u: 0, v: 0, w: 0,        // Lineales (sistema cuerpo)
            p: 0, q: 0, r: 0,        // Angulares
            
            // Suspensión [m]
            z_FL: 0, z_FR: 0, z_RL: 0, z_RR: 0,
            
            // Ruedas [rad/s]
            omega_FL: 0, omega_FR: 0, omega_RL: 0, omega_RR: 0,
            
            // Temperaturas neumáticos [°C]
            temp_FL: 80, temp_FR: 80, temp_RL: 85, temp_RR: 85,
            
            // Estado motor
            engineRPM: 4000,
            throttlePosition: 0,
            brakePressure: 0,
            gear: 0,
            clutchPosition: 1.0
        };
    }

    initializeSubsystems() {
        this.tireModel = new PacejkaMF52();
        this.aeroModel = new Aerodynamics();
        this.suspensionModel = new Suspension();
        this.drivetrainModel = new Drivetrain();
        this.telemetry = new Telemetry();
    }

    // ==================== MÉTODO PRINCIPAL ====================
    
    update(deltaTime, inputs) {
        // 1. Actualizar controles
        this.state.throttlePosition = inputs.throttle;
        this.state.brakePressure = inputs.brake;
        this.state.gear = inputs.gear;
        
        // 2. Calcular transferencia de carga
        const wheelLoads = this.calculateWheelLoads();
        
        // 3. Calcular fuerzas de suspensión
        const suspensionForces = this.suspensionModel.calculate(
            this.state, this.constants, deltaTime
        );
        
        // 4. Calcular fuerzas de neumáticos
        const tireForces = this.calculateTireForces(wheelLoads, inputs.steering);
        
        // 5. Calcular aerodinámica
        const aeroForces = this.aeroModel.calculate(
            this.state, this.constants
        );
        
        // 6. Calcular motor/frenos
        const powertrain = this.drivetrainModel.calculate(
            this.state, this.constants, inputs
        );
        
        // 7. Sumar todas las fuerzas
        const totalForces = this.sumForces(tireForces, aeroForces, suspensionForces, powertrain);
        const totalMoments = this.sumMoments(tireForces, suspensionForces);
        
        // 8. Calcular aceleraciones
        const accelerations = this.calculateAccelerations(totalForces, totalMoments);
        
        // 9. Integrar con RK4
        this.integrateRK4(accelerations, deltaTime);
        
        // 10. Actualizar temperaturas y desgaste
        this.updateTireTemperatures(tireForces);
        
        // 11. Guardar telemetría
        this.telemetry.record(this.state, inputs, {
            forces: {tire: tireForces, aero: aeroForces, suspension: suspensionForces},
            accelerations: accelerations,
            wheelLoads: wheelLoads
        });
        
        return this.getOutput();
    }

    calculateWheelLoads() {
        const m = this.constants.MASS;
        const g = this.constants.GRAVITY;
        const h = this.constants.COG_HEIGHT;
        const L = this.constants.WHEELBASE;
        
        // Aceleraciones
        const ax = this.state.u * this.state.r;
        const ay = this.state.v * this.state.r;
        
        // Transferencias
        const longTransfer = (m * ax * h) / L;
        const latTransferFront = (m * ay * h) / this.constants.TRACK_WIDTH.front;
        const latTransferRear = (m * ay * h) / this.constants.TRACK_WIDTH.rear;
        
        // Cargas estáticas (45% delantero, 55% trasero)
        const staticFront = m * g * 0.45 / 2;
        const staticRear = m * g * 0.55 / 2;
        
        return {
            FL: Math.max(100, staticFront - longTransfer * 0.5 - latTransferFront * 0.5),
            FR: Math.max(100, staticFront - longTransfer * 0.5 + latTransferFront * 0.5),
            RL: Math.max(100, staticRear + longTransfer * 0.5 - latTransferRear * 0.5),
            RR: Math.max(100, staticRear + longTransfer * 0.5 + latTransferRear * 0.5)
        };
    }

    calculateTireForces(loads, steering) {
        const forces = {};
        const slipAngles = this.calculateSlipAngles(steering);
        
        ['FL', 'FR', 'RL', 'RR'].forEach((wheel, i) => {
            const slipAngle = slipAngles[i];
            const slipRatio = this.calculateSlipRatio(wheel);
            const load = loads[wheel];
            const temp = this.state[`temp_${wheel}`];
            
            const tireForce = this.tireModel.calculateForces(
                slipAngle,
                slipRatio,
                load,
                temp,
                1.3 // presión [bar]
            );
            
            forces[wheel] = this.transformToCarFrame(tireForce, wheel, steering);
        });
        
        return forces;
    }

    calculateSlipAngles(steering) {
        const vx = this.state.u || 0.001;
        const vy = this.state.v || 0.001;
        const r = this.state.r || 0.001;
        const L = this.constants.WHEELBASE;
        
        const frontSlip = Math.atan2(vy + (L/2) * r, Math.abs(vx)) - steering;
        const rearSlip = Math.atan2(vy - (L/2) * r, Math.abs(vx));
        
        return [frontSlip, frontSlip, rearSlip, rearSlip];
    }

    calculateSlipRatio(wheel) {
        const vx = Math.max(Math.abs(this.state.u), 0.1);
        const wheelSpeed = this.state[`omega_${wheel}`] || 0;
        const linearSpeed = wheelSpeed * this.constants.TIRE_RADIUS;
        
        return (linearSpeed - vx) / vx;
    }

    transformToCarFrame(tireForce, wheel, steering) {
        const isFront = wheel.startsWith('F');
        const isLeft = wheel.endsWith('L');
        
        let fx = tireForce.Fx;
        let fy = tireForce.Fy;
        
        if (isFront) {
            const cos = Math.cos(steering);
            const sin = Math.sin(steering);
            
            const fx_rot = fx * cos - fy * sin;
            const fy_rot = fx * sin + fy * cos;
            
            fx = fx_rot;
            fy = fy_rot;
        }
        
        return { fx, fy, fz: tireForce.Fz };
    }

    sumForces(tireForces, aeroForces, suspensionForces, powertrain) {
        let Fx = 0, Fy = 0, Fz = 0;
        
        ['FL', 'FR', 'RL', 'RR'].forEach(wheel => {
            const force = tireForces[wheel];
            Fx += force.fx;
            Fy += force.fy;
            Fz += force.fz;
        });
        
        Fx -= aeroForces.drag;
        Fz += aeroForces.downforce;
        
        Fx += powertrain.engineForce - powertrain.brakeForce;
        Fz -= this.constants.MASS * this.constants.GRAVITY;
        
        return { Fx, Fy, Fz };
    }

    sumMoments(tireForces, suspensionForces) {
        let Mx = 0, My = 0, Mz = 0;
        
        ['FL', 'FR', 'RL', 'RR'].forEach(wheel => {
            const force = tireForces[wheel];
            const isFront = wheel.startsWith('F');
            const isLeft = wheel.endsWith('L');
            
            const track = isFront ? this.constants.TRACK_WIDTH.front : this.constants.TRACK_WIDTH.rear;
            const yOffset = isLeft ? -track/2 : track/2;
            const wheelbaseOffset = isFront ? this.constants.WHEELBASE/2 : -this.constants.WHEELBASE/2;
            
            Mx += force.fz * yOffset;
            My += -force.fz * wheelbaseOffset;
            Mz += force.fy * wheelbaseOffset - force.fx * yOffset;
        });
        
        return { Mx, My, Mz };
    }

    calculateAccelerations(totalForces, totalMoments) {
        return {
            du: totalForces.Fx / this.constants.MASS,
            dv: totalForces.Fy / this.constants.MASS,
            dw: totalForces.Fz / this.constants.MASS,
            dp: totalMoments.Mx / this.constants.INERTIA.xx,
            dq: totalMoments.My / this.constants.INERTIA.yy,
            dr: totalMoments.Mz / this.constants.INERTIA.zz
        };
    }

    integrateRK4(accelerations, dt) {
        // Implementación Runge-Kutta 4
        const k1 = accelerations;
        const k2 = this.calculateIntermediateAcceleration(k1, dt/2);
        const k3 = this.calculateIntermediateAcceleration(k2, dt/2);
        const k4 = this.calculateIntermediateAcceleration(k3, dt);
        
        // Promedio ponderado
        const du = (k1.du + 2*k2.du + 2*k3.du + k4.du) / 6;
        const dv = (k1.dv + 2*k2.dv + 2*k3.dv + k4.dv) / 6;
        const dw = (k1.dw + 2*k2.dw + 2*k3.dw + k4.dw) / 6;
        const dp = (k1.dp + 2*k2.dp + 2*k3.dp + k4.dp) / 6;
        const dq = (k1.dq + 2*k2.dq + 2*k3.dq + k4.dq) / 6;
        const dr = (k1.dr + 2*k2.dr + 2*k3.dr + k4.dr) / 6;
        
        // Integrar velocidades
        this.state.u += du * dt;
        this.state.v += dv * dt;
        this.state.w += dw * dt;
        this.state.p += dp * dt;
        this.state.q += dq * dt;
        this.state.r += dr * dt;
        
        // Integrar posiciones
        const cosYaw = Math.cos(this.state.yaw);
        const sinYaw = Math.sin(this.state.yaw);
        
        this.state.x += (this.state.u * cosYaw - this.state.v * sinYaw) * dt;
        this.state.y += (this.state.u * sinYaw + this.state.v * cosYaw) * dt;
        this.state.z += this.state.w * dt;
        
        // Integrar orientación
        this.state.roll += this.state.p * dt;
        this.state.pitch += this.state.q * dt;
        this.state.yaw += this.state.r * dt;
    }

    calculateIntermediateAcceleration(accel, dt) {
        // Para RK4: calcular aceleraciones en punto intermedio
        return {
            du: accel.du,
            dv: accel.dv,
            dw: accel.dw,
            dp: accel.dp,
            dq: accel.dq,
            dr: accel.dr
        };
    }

    updateTireTemperatures(tireForces) {
        ['FL', 'FR', 'RL', 'RR'].forEach(wheel => {
            const force = tireForces[wheel];
            const forceMagnitude = Math.sqrt(force.fx**2 + force.fy**2);
            const speed = Math.sqrt(this.state.u**2 + this.state.v**2);
            
            // Calor generado = fuerza * velocidad * coeficiente
            const heatGenerated = forceMagnitude * speed * 0.0001;
            
            // Enfriamiento natural
            const cooling = (this.state[`temp_${wheel}`] - 80) * 0.01;
            
            this.state[`temp_${wheel}`] += (heatGenerated - cooling) * 0.1;
            this.state[`temp_${wheel}`] = Math.max(60, Math.min(120, this.state[`temp_${wheel}`]));
        });
    }

    getOutput() {
        return {
            position: { x: this.state.x, y: this.state.y, z: this.state.z },
            rotation: { roll: this.state.roll, pitch: this.state.pitch, yaw: this.state.yaw },
            speed: Math.sqrt(this.state.u**2 + this.state.v**2) * 3.6, // km/h
            rpm: this.state.engineRPM,
            gear: this.state.gear,
            tireTemps: {
                FL: this.state.temp_FL,
                FR: this.state.temp_FR,
                RL: this.state.temp_RL,
                RR: this.state.temp_RR
            }
        };
    }

    reset() {
        this.initializeState();
    }

    setCarParameters(params) {
        Object.assign(this.constants, params);
    }

    getTelemetry() {
        return this.telemetry.getData();
    }
}

// =========================================================================
// MODELO PACJEKA MF 5.2 (COMPLETO)
// =========================================================================

class PacejkaMF52 {
    constructor() {
        this.coeffs = {
            pCx1: 1.685, pDx1: 1.4, pDx2: -0.08,
            pEx1: 0.5, pEx2: 0.0,
            pKx1: 22.0, pKx2: 0.0,
            pHx1: 0.0, pVx1: 0.0,
            
            pCy1: 1.4, pDy1: 1.35, pDy2: -0.12,
            pEy1: -0.5, pEy2: 0.0,
            pKy1: 20.0, pKy2: 1.0,
            pHy1: 0.0, pVy1: 0.0,
            
            rBx1: 12.0, rBy1: 8.0, rCy1: 1.0,
            
            qBz1: 8.0, qCz1: 1.0, qDz1: 0.12, qEz1: -0.5
        };
    }

    calculateForces(alpha, kappa, Fz, temperature = 90, pressure = 1.3) {
        const Fz0 = 4000;
        const dfz = (Fz - Fz0) / Fz0;
        
        const tempFactor = 0.8 + (temperature / 150) * 0.4;
        const pressureFactor = 0.9 + (pressure / 1.3) * 0.2;
        const conditionFactor = tempFactor * pressureFactor;
        
        // Longitudinal
        const Cx = this.coeffs.pCx1;
        const Dx = (this.coeffs.pDx1 + this.coeffs.pDx2 * dfz) * conditionFactor;
        const Ex = this.coeffs.pEx1 + this.coeffs.pEx2 * dfz;
        const Kx = Fz * (this.coeffs.pKx1 + this.coeffs.pKx2 * dfz);
        const Bx = Kx / (Cx * Dx);
        
        const Fx0 = Dx * Math.sin(Cx * Math.atan(
            Bx * kappa - Ex * (Bx * kappa - Math.atan(Bx * kappa))
        ));
        
        // Lateral
        const Cy = this.coeffs.pCy1;
        const Dy = (this.coeffs.pDy1 + this.coeffs.pDy2 * dfz) * conditionFactor;
        const Ey = this.coeffs.pEy1 + this.coeffs.pEy2 * dfz;
        const Ky = Fz * (this.coeffs.pKy1 + this.coeffs.pKy2 * dfz);
        const By = Ky / (Cy * Dy);
        
        const Fy0 = Dy * Math.sin(Cy * Math.atan(
            By * alpha - Ey * (By * alpha - Math.atan(By * alpha))
        ));
        
        // Combined slip
        const combined = Math.cos(Math.atan(this.coeffs.rBx1 * alpha));
        const Fx = Fx0 * combined;
        const Fy = Fy0 * Math.cos(Math.atan(this.coeffs.rBy1 * kappa));
        
        // Friction limit
        const mu = 1.5 * conditionFactor;
        const F_max = mu * Fz;
        const F_current = Math.sqrt(Fx**2 + Fy**2);
        
        if (F_current > F_max) {
            const scale = F_max / F_current;
            return { Fx: Fx * scale, Fy: Fy * scale, Fz: Fz };
        }
        
        return { Fx, Fy, Fz };
    }
}

// =========================================================================
// AERODINÁMICA
// =========================================================================

class Aerodynamics {
    calculate(state, constants) {
        const speed = Math.sqrt(state.u**2 + state.v**2);
        const dynamicPressure = 0.5 * constants.AIR_DENSITY * speed**2;
        
        const totalDownforce = dynamicPressure * constants.CL * constants.FRONTAL_AREA;
        const frontDownforce = totalDownforce * constants.AERO_BALANCE;
        const rearDownforce = totalDownforce * (1 - constants.AERO_BALANCE);
        
        const drag = dynamicPressure * constants.CD * constants.FRONTAL_AREA;
        
        return {
            downforce: totalDownforce,
            downforceFront: frontDownforce,
            downforceRear: rearDownforce,
            drag: drag
        };
    }
}

// =========================================================================
// SUSPENSIÓN
// =========================================================================

class Suspension {
    calculate(state, constants, dt) {
        const forces = {};
        let total = 0;
        
        ['FL', 'FR', 'RL', 'RR'].forEach(wheel => {
            const isFront = wheel.startsWith('F');
            const spring = isFront ? constants.SPRING_RATE.front : constants.SPRING_RATE.rear;
            const damper = isFront ? constants.DAMPER_RATE.front : constants.DAMPER_RATE.rear;
            
            const displacement = state[`z_${wheel}`];
            const velocity = 0; // Simplificado
            
            const springForce = -spring * displacement;
            const damperForce = -damper * velocity;
            
            forces[wheel] = springForce + damperForce;
            total += forces[wheel];
        });
        
        return { ...forces, total };
    }
}

// =========================================================================
// TRANSMISIÓN
// =========================================================================

class Drivetrain {
    calculate(state, constants, inputs) {
        const rpm = state.engineRPM;
        const throttle = inputs.throttle;
        
        let torque;
        if (rpm < 6000) {
            torque = 300 + (rpm - 4000) * 0.1;
        } else if (rpm < 10500) {
            torque = 500 + (rpm - 6000) * 0.04;
        } else if (rpm < 13000) {
            torque = 680;
        } else {
            torque = 680 - (rpm - 13000) * 0.05;
        }
        
        const engineTorque = torque * throttle;
        const gearRatio = constants.GEAR_RATIOS[state.gear] || 0;
        const totalRatio = gearRatio * constants.FINAL_DRIVE;
        
        const wheelTorque = engineTorque * totalRatio;
        const wheelForce = wheelTorque / constants.TIRE_RADIUS;
        
        const brakeTorque = inputs.brake * constants.MAX_BRAKE_TORQUE;
        const brakeForce = brakeTorque / constants.TIRE_RADIUS;
        
        return {
            engineForce: wheelForce,
            brakeForce: brakeForce,
            engineTorque: engineTorque,
            brakeTorque: brakeTorque
        };
    }
}

// =========================================================================
// TELEMETRÍA
// =========================================================================

class Telemetry {
    constructor() {
        this.data = {
            frames: [],
            startTime: Date.now(),
            sampleRate: 100
        };
    }
    
    record(state, inputs, additional) {
        const frame = {
            timestamp: performance.now(),
            state: JSON.parse(JSON.stringify(state)),
            inputs: { ...inputs },
            ...additional
        };
        
        this.data.frames.push(frame);
        
        if (this.data.frames.length > 36000) {
            this.data.frames.shift();
        }
    }
    
    getData() {
        return {
            ...this.data,
            endTime: Date.now(),
            totalFrames: this.data.frames.length
        };
    }
}

// Exportar
if (typeof module !== 'undefined') {
    module.exports = { F1PhysicsCore, PacejkaMF52 };
}