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
          <img :src="e.imgSrc" class="image" />
          <div style="padding: 14px">
            <span>{{ e.header }}</span>
            <div class="bottom clearfix">
              <time class="time">{{ e.date }}</time>
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

export default {
  data: function () {
    return {
      elements: [
        {
          header: "Key",
          date: new Date(),
          imgSrc: require("@/assets/images/key.png"),
        },
        {
          header: "Wallet",
          date: new Date(),
          imgSrc: require("@/assets/images/wallet.png"),
        },
        {
          header: "Bag",
          date: new Date(),
          imgSrc: require("@/assets/images/bag.png"),
        },
        {
          header: "Bag",
          date: new Date(),
          imgSrc: require("@/assets/images/bag.png"),
        },
        {
          header: "Bag",
          date: new Date(),
          imgSrc: require("@/assets/images/bag.png"),
        },
        {
          header: "Wallet",
          date: new Date(),
          imgSrc: require("@/assets/images/wallet.png"),
        },
        {
          header: "Bag",
          date: new Date(),
          imgSrc: require("@/assets/images/bag.png"),
        },
        {
          header: "Bag",
          date: new Date(),
          imgSrc: require("@/assets/images/bag.png"),
        },
        {
          header: "Bag",
          date: new Date(),
          imgSrc: require("@/assets/images/bag.png"),
        },
      ],
    };
  },
  methods: {},
  created() {},
};
</script>
<style>
</style>