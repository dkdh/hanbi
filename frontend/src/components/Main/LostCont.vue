<template>
  <el-card
    id="LostCont"
    shadow="always"
    :body-style="{ padding: '20px' }"
    style="overflow-y: auto"
  >
    <div id="lostWrap" style="width: 100%">
      <div>
        <el-card
          :body-style="{ padding: '0px' }"
          v-for="e in elements"
          :key="e"
        >
          <img :src="e.fileUrl" class="image" />
          <div style="padding: 14px">
            <span>{{ e.createdAt }}</span>
            <div class="bottom clearfix">
              <!-- <el-button type="text" class="button">Operating</el-button> -->
            </div>
          </div>
        </el-card>
      </div>
    </div>
  </el-card>
</template>

<script>
import "@/assets/css_kjh/LostCont.css";
import axios from "axios";

export default {
  data: function () {
    return {
      elements: [
      ],
    };
  },
  methods: {
    getLost() {
      axios
        .get("http://j5a102.p.ssafy.io/api/lostItem")
        .then((res) => {
          console.log(res.data)
          this.elements = res.data;
        })
        .catch((err) => {
          console.log(err);
        });
    },
  },
  created() {
    this.getLost()
  },
};
</script>
<style>
</style>