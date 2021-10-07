const mongoose = require('mongoose');
// 배포 시 변경
mongoose.connect("mongodb://hanbi:iot102iot@j5a102.p.ssafy.io:27017/hanbidb");
const db = mongoose.connection;

const handleOpen = () => console.log("✅ Connected to DB");
const handleError = (error) => console.log("❌ DB Error", error);

db.on("error", handleError);
db.once("open", handleOpen);

module.exports = db