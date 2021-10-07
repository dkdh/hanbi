<template>
    <el-card id="LostCont" shadow="always" :body-style="{ padding: '20px' }">
        <el-row>
            <el-col :span="8" v-for="(video, idx) in videos" :key="idx">
            <el-card style="margin-bottom: 30px">
              <video  width="320" height="240" :src="video.fileUrl" class="image" controls>
              </video>
              <div style="padding: 14px">
                <span>{{ video.createdAt }}</span>
              </div>
            </el-card>
            </el-col>
        </el-row>
    </el-card>
    <!-- <el-card id="LostCont" shadow="always" :body-style="{ padding: '20px' }">
        <div id="lostWrap">
            <el-card
            v-for="(video, idx) in test"
            :key="idx"
            :span="8"
            >
            <video  width="320" height="240" :src="video.fileUrl" class="image" controls>
            </video>
            <div style="padding: 14px">
            <span>{{ video.createdAt }}</span>
            </div>
            </el-card>
        </div>
    </el-card> -->
</template>

<script>
import "@/assets/css/LostCont.css";
import axios from "axios"

export default {
  data: function() {
    return {
      videos : [],
    }
  },
  methods: {
    getBlackBox() {
      axios.get("http://j5a102.p.ssafy.io/api/record")
      .then(res => {
        console.log(res)
        console.log(res.data)
        this.videos = res.data.records
        console.log(this.videos)
      })
      .catch(err => {
        console.log(err)
      })
    },
  },
  created() {
    this.getBlackBox()
  },


};
</script>
<style>
</style>