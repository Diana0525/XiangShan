package xiangshan.cache.prefetch

import org.chipsalliance.cde.config.{Parameters, Field}
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.cache._
import xiangshan.cache.mmu.{HasTlbConst}
import utils._
import utility._

case object ACDPParamsKey extends Field[ACDPParameters]

case class ACDPParameters(
    cmTableEntries: Int,
    cmTagWidth: Int,
    prefetchDepthThreshold: Int,
    firstLevelPageNumHighBits: Int,
    firstLevelPageNumLowBits: Int,
    secondLevelPageNumHighBits: Int,
    secondLevelPageNumLowBits: Int,

    blockBytes: Int,
    nEntries: Int
)
{
    def totalWidth = log2Up(nEntries) // id's width
}

class CacheMissTableEntry (implicit p: Parameters) extends PrefetchBundle {
  val addrHighBits  = UInt(acdpParams.cmTagWidth.W)

  def apply(addrHighBits: UInt) = {
    val entry = Wire(new CacheMissTableEntry)
    entry.addrHighBits := addrHighBits
    entry
  }
  override def toPrintable: Printable = { p"${addrHighBits}}" }
}

class TestMissAddressReq(implicit p: Parameters) extends PrefetchBundle {
  // find whether miss address is in cache miss table
  val missAddr = UInt(PAddrBits.W)
  val testAddr = UInt(acdpParams.cmTagWidth.W) 
  val ptr = UInt(log2Up(acdpParams.cmTableEntries).W) // index of address high bists in table

  override def toPrintable: Printable = {
    p"missAddr=0x${Hexadecimal(missAddr)} testAddr=${testAddr} ptr=${ptr}"
  }
}

class TestMissAddressResp(implicit p: Parameters) extends PrefetchBundle {
  val testAddr = UInt(acdpParams.cmTagWidth.W) 
  val ptr = UInt(log2Up(acdpParams.cmTableEntries).W) 
  val hit = Bool()

  override def toPrintable: Printable = {
    p"pff=${testAddr} ptr=${ptr} hit=${hit}"
  }
}

class TestMissAddressBundle(implicit p: Parameters) extends PrefetchBundle {
    val req = DecoupledIO(new TestMissAddressReq)
    val resp = Flipped(DecoupledIO(new TestMissAddressResp))

    override def toPrintable: Printable = {
        p"req: v=${req.valid} r=${req.ready} ${req.bits} " +
        p"resp: v=${resp.valid} r=${resp.ready} ${resp.bits}"
  }
}

class AdvanceContentDirectedPrefetchReq(implicit p: Parameters) extends PrefetchReq {
  val id = UInt(acdpParams.totalWidth.W)

  override def toPrintable: Printable = {
    p"addr=0x${Hexadecimal(addr)} w=${write} id=0x${Hexadecimal(id)}"
  }
}

class AdvanceContentDirectedPrefetchResp(implicit p: Parameters) extends PrefetchResp {
  val id = UInt(acdpParams.totalWidth.W)

  override def toPrintable: Printable = {
    p"id=0x${Hexadecimal(id)}"
  }
}

class AdvanceContentDirectedPrefetchFinish(implicit p: Parameters) extends PrefetchFinish {
  val id = UInt(acdpParams.totalWidth.W)

  override def toPrintable: Printable = {
    p"id=0x${Hexadecimal(id)}"
  }
}

class AdvanceContentDirectedPrefetchIO(implicit p: Parameters) extends PrefetchBundle {
  val train = Flipped(ValidIO(new PrefetchTrain))
  val req = DecoupledIO(new AdvanceContentDirectedPrefetchReq)
  val resp = Flipped(DecoupledIO(new AdvanceContentDirectedPrefetchResp))
  val finish = DecoupledIO(new AdvanceContentDirectedPrefetchFinish)

  override def toPrintable: Printable = {
    p"train: v=${train.valid} ${train.bits} " +
      p"req: v=${req.valid} r=${req.ready} ${req.bits} " +
      p"resp: v=${resp.valid} r=${resp.ready} ${resp.bits} " +
      p"finish: v=${finish.valid} r=${finish.ready} ${finish.bits}"
  }
}

class RecentCacheMissTable(implicit p: Parameters) extends PrefetchModule {
    val io = IO(new Bundle {
        val w = Flipped(DecoupledIO(UInt(PAddrBits.W)))
        val r = Flipped(new TestMissAddressBundle)
  })
  // RCM table is direct mapped, accessed through high 18 bits of address,
  // each entry holding high 18 bits of address.

  def cmTagWidth = acdpParams.cmTagWidth
  def cmTableEntries = acdpParams.cmTableEntries
  def firstLevelPageNumHighBits = acdpParams.firstLevelPageNumHighBits
  def secondLevelPageNumLowBits = acdpParams.secondLevelPageNumLowBits
  def tag(addr: UInt) = addr(firstLevelPageNumHighBits,secondLevelPageNumLowBits) 
  def cmTableEntry() = new Bundle {
    val valid = Bool()
    val addrHighBits  = UInt(cmTagWidth.W)

    override def toPrintable: Printable = {
      p"${valid} ${Hexadecimal(addrHighBits)}"
    }
  }
  // record cmTableEntries miss addr: |  ......  |  18-bit addr1  |  18-bit addr2  | ...... |
  val rcmTable = Module(new SRAMTemplate(cmTableEntry(), set = cmTableEntries, way = 1, shouldReset = true, singlePort = true))

  val wAddr = io.w.bits// TODO: how to make sure this is miss address?
  rcmTable.io.w.req.valid := io.w.valid && !io.r.req.valid
  rcmTable.io.w.req.bits.setIdx := tag(wAddr)
  rcmTable.io.w.req.bits.data(0).valid := true.B
  rcmTable.io.w.req.bits.data(0).addrHighBits := tag(wAddr)

  val rAddr = io.r.req.bits.missAddr
  val rData = Wire(cmTableEntry())
  rcmTable.io.r.req.valid := io.r.req.fire
  rcmTable.io.r.req.bits.setIdx := rAddr
  rData := rcmTable.io.r.resp.data(0)

  val rwConflict = io.w.fire && io.r.req.fire
  assert(!RegNext(rwConflict), "single port SRAM should not read and write at the same time")

  io.w.ready := rcmTable.io.w.req.ready && !io.r.req.valid
  io.r.req.ready := true.B
  io.r.resp.valid := RegNext(rcmTable.io.r.req.fire)
  io.r.resp.bits.testAddr := RegNext(io.r.req.bits.testAddr)
  io.r.resp.bits.ptr := RegNext(io.r.req.bits.ptr)
  io.r.resp.bits.hit := rData.valid && rData.addrHighBits === RegNext(tag(rAddr))

   // debug info
  XSDebug(io.w.fire, p"io.write: v=${io.w.valid} addr=0x${Hexadecimal(io.w.bits)}\n")
  XSDebug(p"io.read: ${io.r}\n")
  XSDebug(io.w.fire, p"wAddr=0x${Hexadecimal(wAddr)} idx=${Hexadecimal(tag(wAddr))} tag=${Hexadecimal(tag(wAddr))}\n")
  XSDebug(io.r.req.fire, p"rAddr=0x${Hexadecimal(rAddr)} idx=${Hexadecimal(tag(rAddr))} rData=${rData}\n")
}

class PointerDataRecognition(implicit p: Parameters) extends PrefetchModule {
  val io = IO(new Bundle {
    val respFromL3 = Flipped(DecoupledIO(UInt(PAddrBits.W))) // resp block from L3
    val hitInL2 = Flipped(DecoupledIO(UInt(PAddrBits.W))) // hit block in L2
    val pointerAddr = Output(UInt(PAddrBits.W)) // data of pointer
    val test = new TestMissAddressBundle
  })

  
}