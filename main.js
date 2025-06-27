let navLinks = document.getElementById("links");
function openCloseMenu() {
  navLinks.classList.toggle("active");
}
document.getElementById("close-menu").addEventListener("click", function () {
  navLinks.classList.remove("active");
});

function sendPackage(size) {
  fetch("http://localhost:3000/send-package", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ size }),
  })
    .then((res) => res.json())
    .then((data) => {
      if (data.success) {
        alert("✅ تم إرسال الطلب: " + data.size);
      } else {
        alert("❌ فشل في إرسال الطلب");
      }
    })
    .catch((err) => {
      console.error("❌ Error sending request:", err);
      alert("⚠️ حدث خطأ أثناء الإرسال");
    });
}
