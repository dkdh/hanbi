const express = require("express")
const router = express.Router()
const mapService = require("../service/map")
const lostService = require("../service/lost")
const historyService = require("../service/history")

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

module.exports = router