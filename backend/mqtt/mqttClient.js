import mqtt from "mqtt";

const host =
  process.env.MQTT_HOST ||
  "d17d7e2732244dcaa790629f8bf4296c.s1.eu.hivemq.cloud";
const port = process.env.MQTT_PORT || 8883;
const user = process.env.MQTT_USER || "sadeen_hh";
const pass = process.env.MQTT_PASS || "SqtjRzEZYU96khY";

const connectUrl = `mqtts://${host}:${port}`;

const client = mqtt.connect(connectUrl, {
  username: user,
  password: pass,
});

client.on("connect", () => {
  console.log("📡 Connected to HiveMQ Cloud");
});

client.on("error", (err) => {
  console.error("❌ MQTT Connection Error:", err);
});

export default client;
