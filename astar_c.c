#include <stdlib.h>
#include <string.h>

/* ── Min-Heap ───────────────────────────────────────────────── */
typedef struct {
    int f, g, idx;
} HeapNode;

typedef struct {
    HeapNode *data;
    int size, cap;
} Heap;

static void heap_init(Heap *h, int cap) {
    h->data = (HeapNode *)malloc(cap * sizeof(HeapNode));
    h->size = 0;
    h->cap = cap;
}

static void heap_push(Heap *h, int f, int g, int idx) {
    if (h->size >= h->cap) return;
    int i = h->size++;
    h->data[i].f = f; h->data[i].g = g; h->data[i].idx = idx;
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (h->data[i].f < h->data[p].f) {
            HeapNode tmp = h->data[i]; h->data[i] = h->data[p]; h->data[p] = tmp;
            i = p;
        } else break;
    }
}

static HeapNode heap_pop(Heap *h) {
    HeapNode top = h->data[0];
    h->data[0] = h->data[--h->size];
    int i = 0;
    for (;;) {
        int l = 2*i+1, r = 2*i+2, s = i;
        if (l < h->size && h->data[l].f < h->data[s].f) s = l;
        if (r < h->size && h->data[r].f < h->data[s].f) s = r;
        if (s != i) {
            HeapNode tmp = h->data[i]; h->data[i] = h->data[s]; h->data[s] = tmp;
            i = s;
        } else break;
    }
    return top;
}

static void heap_free(Heap *h) { free(h->data); }

/* ── A* 2D ──────────────────────────────────────────────────── */

/*
 * blocked: row-major uint8 grid (1=blocked, 0=free), size w*h
 * start_c, start_r, goal_c, goal_r: coordinates
 * out_path: caller-allocated int array (size >= w*h*2), filled with c0,r0,c1,r1,...
 * returns: path length (number of positions), or 0 if no path
 */
__declspec(dllexport)
int astar2d(const unsigned char *blocked, int w, int h,
            int start_c, int start_r, int goal_c, int goal_r,
            int *out_path, int max_path) {

    int total = w * h;
    int *g_arr = (int *)malloc(total * sizeof(int));
    int *parent = (int *)malloc(total * sizeof(int));

    memset(g_arr, -1, total * sizeof(int));
    memset(parent, -1, total * sizeof(int));

    int si = start_r * w + start_c;
    int gi = goal_r * w + goal_c;

    if (blocked[si] || blocked[gi]) {
        free(g_arr); free(parent);
        return 0;
    }

    g_arr[si] = 0;

    Heap heap;
    heap_init(&heap, total > 65536 ? total : 65536);

    int dx = abs(start_c - goal_c) + abs(start_r - goal_r);
    heap_push(&heap, dx, 0, si);

    static const int dc[4] = {0, 0, 1, -1};
    static const int dr[4] = {1, -1, 0, 0};

    int found = 0;

    while (heap.size > 0) {
        HeapNode node = heap_pop(&heap);
        int c = node.idx % w;
        int r = node.idx / w;

        if (g_arr[node.idx] >= 0 && node.g > g_arr[node.idx])
            continue;

        if (node.idx == gi) {
            found = 1;
            break;
        }

        for (int i = 0; i < 4; i++) {
            int nc = c + dc[i];
            int nr = r + dr[i];
            if (nc < 0 || nc >= w || nr < 0 || nr >= h) continue;
            int ni = nr * w + nc;
            if (blocked[ni]) continue;
            int ng = node.g + 1;
            if (g_arr[ni] < 0 || ng < g_arr[ni]) {
                g_arr[ni] = ng;
                parent[ni] = node.idx;
                int hval = abs(nc - goal_c) + abs(nr - goal_r);
                heap_push(&heap, ng + hval, ng, ni);
            }
        }
    }

    int path_len = 0;
    if (found) {
        /* Reconstruct backwards */
        int cur = gi;
        int count = 0;
        while (cur >= 0) { count++; cur = parent[cur]; }
        path_len = count;

        if (count <= max_path) {
            cur = gi;
            for (int i = count - 1; i >= 0; i--) {
                out_path[i * 2]     = cur % w;
                out_path[i * 2 + 1] = cur / w;
                cur = parent[cur];
            }
        } else {
            path_len = 0;
        }
    }

    heap_free(&heap);
    free(g_arr);
    free(parent);
    return path_len;
}
