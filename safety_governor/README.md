# Faraday Safety Governor

**AI-powered safety and motion governance for humanoid robots.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Status](https://img.shields.io/badge/Status-In%20Development-orange)]()

---

# Faraday Safety Governor (Optional)

This module provides an optional, software-only safety and motion governance layer
for ROS2-based Unitree robots.

- No hardware modification
- No replacement of Unitree SDK
- Consumes existing ROS2 topics (IMU, odom, scan)
- Designed for professional training and research deployments

Vendor-agnostic. Optional. Non-intrusive.

## 📖 About

Faraday Robotics builds **vendor-agnostic AI software layers** that optimize **motion, behavior and adaptive efficiency** in humanoid robots.

Rather than replacing hardware controllers or low-level SDKs, Faraday integrates **on top of existing ROS2 stacks**, consuming IMU, SLAM and perception data to improve how robots move, adapt and perform in real-world conditions.

Safety is a **foundational capability** of the Faraday stack, but the broader goal is **motion optimization**: making humanoid robots **faster, smoother, more stable and more precise**, with measurable performance gains before and after integration.

This repository contains the **OSS Lite v1.0** release of the Faraday Safety Governor — the first building block of the Faraday Robotics software stack.

## ❓ Why Faraday Safety Governor?

Humanoid robot manufacturers provide excellent hardware, low-level control and simulation tools.  
What is often missing is a **high-level software layer** that continuously governs motion quality across sensors and contexts.

The Faraday Safety Governor addresses this gap by introducing a **real-time motion governance layer** that:

- Correlates **IMU, SLAM and proximity data** instead of treating them in isolation
- Dynamically scales velocity to maintain **stability, precision and control**
- Prevents unstable behaviors before they become failures
- Provides **measurable motion metrics** usable for optimization and training

While safety constraints are enforced, the Safety Governor is primarily designed as an **optimization wedge**: a foundation upon which more advanced behavior orchestration, training scenarios and compliance layers can be built.

It is the first step toward turning affordable humanoid robots into **professionally deployable systems**.

---

## 🎯 What Does It Do?

The Safety Governor monitors robot state in real-time and dynamically adjusts motion to prevent:

- **Falls** - Monitors tilt/acceleration from IMU
- **Collisions** - Tracks obstacle proximity
- **Localization loss** - Detects SLAM degradation
- **Unstable movements** - Enforces safe velocity limits

### Key Features

- **Real-time velocity scaling** based on safety conditions
- **Multi-sensor fusion** (IMU, SLAM, proximity sensors)
- **Configurable safety profiles** (conservative, normal, aggressive)
- **Audit trail & logging** for compliance
- **ROS2-native** - Works via WebSocket bridge (rosbridge)
- **Vendor-agnostic** - Compatible with Unitree, UBTech, and others
- **Web dashboard** - Real-time monitoring interface

---

## 🏗️ Architecture

```
┌─ FARADAY SAFETY GOVERNOR ────────────┐
│  • Motion governance                 │
│  • Cross-sensor intelligence         │
│  • Safety rule enforcement           │
│  • Audit & logging                   │
└──────────────────────────────────────┘
           ↓ (vendor-agnostic)
┌─ ROS2 / SLAM / SDK ──────────────────┐
│  • Hardware abstraction              │
│  • Sensor drivers                    │
│  • Basic control loops               │
└──────────────────────────────────────┘
           ↓
┌─ ROBOT HARDWARE ─────────────────────┐
│  • Unitree Go2/H1/G1/B2              │
│  • UBTech Walker X                   │
│  • Any ROS2 humanoid                 │
└──────────────────────────────────────┘
```

See [ARCHITECTURE.md](docs/ARCHITECTURE.md) for technical details.

---

## 🚀 Quick Start

### Dashboard (Web Interface)

```bash
cd dashboard
npm install
npm run dev
```

The dashboard provides real-time monitoring with Demo Mode (simulated sensors) and Production Mode (real ROS2 connection via WebSocket).

See [dashboard/README.md](dashboard/README.md) for configuration details.

---

## 🗺️ Roadmap

Faraday Safety Governor is the first of multiple products in the Faraday Robotics stack.

| Product | Status | Purpose |
|---------|--------|---------|
| **Safety Governor** | 🟢 v1.0 (Active) | Real-time safety & motion governance |
| Safety Governor PRO | 🔵 Phase 1b | Recovery behaviors, certified logs, SLA |
| Embodied AI Runtime | 🔵 Phase 2 | Behavior orchestration & decision layer |
| Training Engine | 🔵 Phase 3 | Scenario management, AAR, instructor tools |
| Compliance Layer | 🔵 Phase 4 | Certified logs, regulatory profiles, audit |

See [ROADMAP.md](ROADMAP.md) for the complete product roadmap.

---

## 📚 Documentation

- **[VISION.md](VISION.md)** - Why Faraday Robotics exists
- **[ROADMAP.md](ROADMAP.md)** - Product roadmap and phases
- **[ARCHITECTURE.md](docs/ARCHITECTURE.md)** - Technical architecture
- **[WHY_FARADAY.md](docs/WHY_FARADAY.md)** - The gap we're filling

---

## 🤝 Contributing

We welcome contributions to the OSS Lite version!

- Report bugs via [GitHub Issues](https://github.com/faraday-roboticslab/faraday-safety-governor/issues)
- Submit pull requests for improvements
- Share your use cases and feedback

**Note:** PRO features are closed-source and not accepting external contributions.

---

## 📜 License

**OSS Lite:** MIT License - See [LICENSE](LICENSE) file

**PRO Version:** Commercial license - Contact us for terms

---

## 🌐 About Faraday Robotics

Faraday Robotics builds AI-powered software layers that make humanoid robots safer, more reliable, and production-ready for professional deployments.

**We don't build hardware. We optimize how robots move, behave, and perform.**

- 🌐 Website: [faradaylab.fr](https://faradaylab.fr)
- 📧 Email: contact@faradaylab.fr
- 📍 Location: Paris, France 🇫🇷

---

## 🏆 Credits

**Built by:**
- **William Elong** - Founder & CEO, Faraday Robotics
- **GPT-5** - Safety logic & architecture
- **Claude (Anthropic)** - ROS2 integration & system design  
- **Figma Make** - Dashboard UI & visualization

---

<div align="center">

**Faraday Safety Governor**

*Making humanoid robots safer and production-ready.*

🇫🇷 Made in Paris | 🤖 Works with any ROS2 robot | 🛡️ Safety-first design

[Get Started](#-quick-start) • [Documentation](#-documentation) • [Contact Us](mailto:contact@faradaylab.fr)

</div>
