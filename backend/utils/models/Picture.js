const mongoose = require('mongoose')

const pictureSchema = new mongoose.Schema({
    fileUrl: { type: String, required: true},
    createdAt: { type: String, required: true },
    caseString: { type: String, required: true },
})

const Picture = mongoose.model("Picture", pictureSchema)

module.exports = Picture