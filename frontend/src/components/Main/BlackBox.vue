<template>
    <el-card id="LostCont" shadow="always" :body-style="{ padding: '20px' }">
        <el-row>
            <el-col :span="6" v-for="(video, idx) in videos" :key="idx" style="margin-right:110px">
                <video  width="320" height="240" :src="video.fileUrl" class="image" controls>
                </video>
                <div style="padding: 15px">
                  <span>{{ video.createdAt }}</span>
                </div>
            </el-col>
        </el-row>
    </el-card>
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
        this.videos = res.data
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