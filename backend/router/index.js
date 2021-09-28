const express = require("express")
const router = express.Router()
const mapService = require("../service/map")

router.get("/map", async (req, res) => {

    await mapService.map(req)
        .then((data) => {
            res.writeHead(200, { "Context-Type": "image/jpg" });//보낼 헤더를 만듬
            res.write(data);   //본문을 만들고
            return res.json({ data })

        })
        .catch((error) => {
            return res.json({ error })
        })
})

module.exports = router