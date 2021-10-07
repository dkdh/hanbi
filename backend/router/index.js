const express = require("express")
const router = express.Router()
const mapService = require("../service/map")
const lostService = require("../service/lost")
const historyService = require("../service/history")

const Record = require("../utils/models/Record");
const Lost = require("../utils/models/Lost");
const Picture = require("../utils/models/Picture");


router.get("/map", async (req, res) => {

    await mapService.get_map(req)
        .then((data) => {
            res.writeHead(200, { "Context-Type": "image/jpg" });//보낼 헤더를 만듬
            res.write(data);   //본문을 만들고
            return res.json({ data })
        })
        .catch((error) => {
            return res.json({ error })
        })
})

router.get("/lost", async (req, res) => {
    await lostService.get_lost(req)
        .then((data) => {
            return res.json({ data })
        })
        .catch((error) => {
            return res.json({ error })
        })
})

//todo history 데이터 저장
router.post("/history", async (req, res) => {
    await historyService.post_history(req)
        .then((data) => {
            return res.json({ data })
        })
        .catch((error) => {
            return res.json({ error })
        })
})

// history 데이터 조회
router.get("/history", async (req, res) => {
    await historyService.get_history(req)
        .then((data) => {
            return res.json({ data })
        })
        .catch((error) => {
            return res.json({ error })
        })
})

// 블랙박스 영상 조회
router.get("/record", async (req, res) => {
    try {
        const records = await Record.find({}).sort({ _id: -1 })
        res.send(records)
      } catch (error) {
        console.log(error);
      }
})

// 분실물 조회
router.get("/lost", async (req, res) => {
    try {
        const lostItems = await Lost.find({}).sort({ _id: -1 })
        res.send(lostItems)
      } catch (error) {
        console.log(error);
      }
})

// 사진 조회
router.get("/picture", async (req, res) => {
    try {
        const pictures = await Picture.find({}).sort({ _id: -1 })
        res.send(pictures)
      } catch (error) {
        console.log(error);
      }
})

module.exports = router