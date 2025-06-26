import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import mqttClient from "./mqtt/mqttClient.js";

dotenv.config(); // لتحميل متغيرات .env

console.log("MQTT_HOST:", process.env.MQTT_HOST);
console.log("MQTT_PORT:", process.env.MQTT_PORT);
console.log("MQTT_USER:", process.env.MQTT_USER);

const app = express();
const port = 3000;
const topic = "machine/package-size";

app.use(cors());
app.use(express.json());

// POST /send-package → ينشر رسالة MQTT بناءً على الحجم
app.post("/send-package", (req, res) => {
  const { size } = req.body;

  if (!size) {
    return res.status(400).json({ error: "Package size is required" });
  }
  mqttClient.publish(topic, size, (err) => {
    if (err) {
      console.error("❌ MQTT Publish Error:", err);
      return res.status(500).json({ error: "Failed to send MQTT message" });
    }

    console.log("✅ Package size sent:", size);
    res.json({ success: true, size });
  });
});

// تشغيل السيرفر
app.listen(port, () => {
  console.log(`🚀 Server is running at http://localhost:${port}`);
});
