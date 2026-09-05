// license:BSD-3-Clause
// copyright-holders:Jesus Ramos

#ifndef MAME_CPU_SH_SH7709S_H
#define MAME_CPU_SH_SH7709S_H

#include "sh4.h"

// When enabled switches from icache sampling + heuristic to full icache tracking
// Heavy on CPU usage but useful for debugging icache penalties if your cpu can handle it
#define SH7709S_ICACHE_TRACKING_HEAVY (0)

// Sparse instruction-fetch miss approximation
// Instruction fetches are only sampled at data accesses, so this estimates the
// IF misses the fetch stream incurred between samples from the cache residency
// of a small window of lines around the sampled pc, scaled by the cycle interval
// since the previous sample.

// Number of 16-byte cache lines around the sampled pc checked for residency
#define SH7709S_IF_WINDOW_LINES (8)
// Window residency % at or above which the fetch stream is treated as hitting
#define SH7709S_IF_RESIDENT_THRESHOLD_PCT (75)
// Cap on the per-interval IF penalty as a multiple of the interval length
#define SH7709S_IF_PENALTY_MAX_SCALE (1)
// Safety net to maintain save state compat while testing
#define SH7709S_IF_MAX_INTERVAL (1000000)


// SH7709S has a unified cache which means that not all cache hits are "free"
// Cache hits while the fill is happening stall on fill wait (easy to handle).
// Cache hits while an instruction fetch need to happen stall the instruction fetch,
// this is where the trouble starts. As sampling every PC access + adding instrumentation
// for branch predictor and other bits is quite hefty we instead do some approximations

// On these cache hits we calculate a budget over an interval for amount of "free"
// cache hits we're allowed. Once that budget is consumed we start considering any
// further ones that could have coincided with instruction fetches to stall for a cycle.
// This models the single port unified cache contention

// Cap on the accumulated free port slots
#define SH7709S_HIT_BUDGET_MAX (4)

// U bit tracked in the dirty field, V bit currently untracked
struct sh7709s_cache_entry
{
	uint32_t tag; // Address tag for entry
	uint8_t lru; // LRU value [0, SH7709S_CACHE_ASSOCIATIVITY)
	uint8_t dirty; // Dirty bit for write eviction
};

class sh7709s_device : public sh3_base_device
{
public:
	sh7709s_device(const machine_config& mconfig, const char* tag, device_t* owner, uint32_t clock, endianness_t endianness = ENDIANNESS_LITTLE);

	// DRC functions used to update the cache state
	void drc_memory_access_read();
	void drc_memory_access_write();

	void update_access_cycles(uint32_t address, bool write);

#if SH7709S_ICACHE_TRACKING_HEAVY == 1
	void drc_update_icache();
	bool generate_opcode(drcuml_block& block, compiler_state& compiler, const opcode_desc* desc, uint32_t ovrpc) override;
#endif

protected:
	virtual void sh3_register_map(address_map& map) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void static_generate_memory_accessor(int size, int iswrite, const char* name, uml::code_handle*& handleptr) override;

	virtual uint32_t ccr_r(offs_t offset, uint32_t mem_mask) override;
	virtual void ccr_w(offs_t offset, uint32_t data, uint32_t mem_mask) override;

	void cache_7709s_map(address_map& map);
	uint32_t cache_address_array_r(offs_t offset, uint32_t mem_mask);
	void cache_address_array_w(offs_t offset, uint32_t data, uint32_t mem_mask);

private:
	static constexpr uint32_t SH7709S_CACHE_SIZE = 16384;
	static constexpr uint32_t SH7709S_CACHE_LINE_SIZE = 16;
	static constexpr uint32_t SH7709S_CACHE_ENTRY_COUNT = (SH7709S_CACHE_SIZE / SH7709S_CACHE_LINE_SIZE);
	static constexpr uint32_t SH7709S_CACHE_ASSOCIATIVITY = 4;
	static constexpr uint32_t SH7709S_CACHE_BLOCKS = (SH7709S_CACHE_ENTRY_COUNT / SH7709S_CACHE_ASSOCIATIVITY);
	// Cache state tracking
	struct sh7709s_cache_entry m_cache[SH7709S_CACHE_BLOCKS][SH7709S_CACHE_ASSOCIATIVITY];
	uint32_t m_wb_address; // writeback buffer address if there's a dirty cache line to evict
	uint8_t m_last_area_accessed; // last memory area accessed for WCR1 timing purposes
	bool m_last_area_accessed_was_write; // last memory area accessed operation also for WCR1 timing purposes
	unsigned int m_wb_active_cycles; // Track any background cycles for writeback and precharge waits on the same bank
	unsigned int m_last_sdram_bank; // Last accessed sdram bank, used to track when to have to pay tpc(precharge) cost
	unsigned int m_precharge_remaining_cycles;
	unsigned int m_burst_continuation_remaining_cycles; // Remaining burst words still occupying the bus after the critical word lands
	uint64_t m_last_op_cycle_count; // Track the last cycle we did a memory operation for background accounting
	// Sparse IF miss approximation state
	uint64_t m_last_if_sample_cycles; // total_cycles() at the last data-access sample
	uint64_t m_last_if_sample_penalty; // cumulative memory penalty at last data-access sample
	uint64_t m_cum_mem_penalty; // cumulative memory-access penalty charged
	int m_hit_port_budget; // free single-port slots available to data hits

	bool cache_access(uint32_t address, bool write);
	unsigned int access_penalty(uint32_t address, bool write, bool *bus_op, bool data_access);
	bool cache_resident(uint32_t address);
	uint32_t if_miss_cost(uint32_t address);
	uint64_t if_sample_interval();
	void refill_hit_port_budget(uint64_t interval_cycles);
	uint32_t approximate_if_miss_penalty(uint32_t pc, uint64_t interval_cycles);

	// Timing calculation/decode related functions
	uint32_t get_wcr1_timing(uint32_t area);
	uint32_t get_wcr2_timing(uint32_t address);
	uint32_t mcr_tpc();
	uint32_t mcr_rcd();
	uint32_t mcr_trwl();
	uint32_t mcr_tras();
	uint32_t cache_line_fetch_count(uint32_t address);
	uint32_t sdram_bank(uint32_t address);
};

DECLARE_DEVICE_TYPE(SH7709S, sh7709s_device)

#endif /* MAME_CPU_SH7709S_H */
