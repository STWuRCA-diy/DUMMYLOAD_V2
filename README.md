DUMMYLOAD_V2

High-Voltage Electronic Dummy Load (Non-Commercial, Open-Source)

DUMMYLOAD_V2 is an open-source, non-commercial electronic dummy load designed for testing high-voltage linear power supplies, especially those used in tube (valve) amplifier circuits.

The project includes hardware (PCB), firmware, and mechanical enclosure, forming a complete DIY measurement and load solution for HV power supply development and testing.

⸻

⚡ Intended Use
	•	Testing high-voltage linear power supplies
	•	Tube amplifier PSU development and diagnostics
	•	Load testing during PSU design, repair, or verification
	•	Educational and laboratory use (DIY / non-commercial)

⸻

🔧 Key Features
	•	Adjustable constant-current load
	•	High-voltage measurement via resistor divider
	•	Current measurement using INA219
	•	Real-time voltage & current display (I²C LCD)
	•	Software-assisted smoothing and caching of measurements
	•	Protection logic implemented in firmware
	•	Stand-alone operation (no PC required)

⸻

📊 Technical Overview (from firmware & design)

Electrical Characteristics
	•	Load mode: Constant Current (CC)
	•	Current regulation range:
0 – ~400 mA (firmware-controlled, step-based)
	•	Voltage measurement:
Indirect (HV resistor divider → INA219 measurement range)
	•	Current sensing:
INA219 (I²C)

⚠️ Note: Maximum voltage capability depends on PCB layout, resistor ratings, MOSFETs, and cooling, not only firmware.

⸻

Firmware Highlights
	•	Measurement value caching to reduce I²C traffic
	•	Time-based refresh for voltage & current readings
	•	LCD update only when content changes (flicker-free)
	•	Designed for stable operation under noisy HV conditions
⚠️ HIGH VOLTAGE WARNING

This project is intended for experienced users.

High voltages used in tube power supplies can be lethal.
	•	Do NOT work on the circuit while powered
	•	Always discharge capacitors
	•	Use proper insulation and creepage distances
	•	Never touch the circuit during operation
	•	The author assumes no responsibility for misuse

If you are not comfortable working with HV — do not build or use this device.

⸻

📜 License (Very Important)

🔒 Non-Commercial Only

This entire project is licensed under:

Creative Commons Attribution-NonCommercial 4.0 International
(CC BY-NC 4.0)

✔ You MAY:
	•	build it for yourself
	•	modify it
	•	share it
	•	learn from it

❌ You MAY NOT:
	•	sell PCBs
	•	sell kits
	•	sell assembled devices
	•	use it commercially in any form

See LICENSE file for details.

⸻

🍺 Friendly Note

If you ever meet me in person and find this project useful,
you owe me a beer.

If we happen to be in a country where it is legal,
a joint is also acceptable 🙂

This is a friendly gesture — not a legal obligation.

⸻

❤️ Credits

Designed and shared by STWuRCA-diy
DIY spirit, tube amps & open hardware 🤘
