#import <Cocoa/Cocoa.h>
#include <dynquora/physics.h>

static constexpr int kCount = 24;
static constexpr CGFloat kScale = 90.0;

@interface DQView : NSView {
  dq_context* context_;
  dq_particle_id ids_[kCount];
  BOOL paused_;
}
@end

@implementation DQView
- (BOOL)acceptsFirstResponder { return YES; }
- (void)resetScene {
  if (context_) dq_context_destroy(context_);
  dq_context_desc desc{};
  desc.struct_size = sizeof desc; desc.max_particles = kCount; desc.max_distance_constraints = kCount - 1;
  desc.solver_iterations = 12; desc.substeps = 2; desc.gravity = {0.0f, -9.81f, 0.0f}; desc.linear_damping = 0.002f; desc.grid_cell_size = 0.5f; desc.enable_floor = 1; desc.floor_height = -2.5f; desc.floor_restitution = 0.1f; desc.floor_friction = 0.15f;
  if (dq_context_create(&desc, &context_) != DQ_OK) return;
  for (int i = 0; i < kCount; ++i) {
    dq_particle_desc p{}; p.position = {-3.0f + i * 0.32f, 2.0f, 0.0f}; p.inverse_mass = i == 0 ? 0.0f : 1.0f;
    dq_context_add_particle(context_, &p, &ids_[i]);
  }
  for (int i = 1; i < kCount; ++i) { dq_distance_constraint_desc c{ids_[i - 1], ids_[i], 0.32f, 0.0f}; dq_constraint_id id; dq_context_add_distance_constraint(context_, &c, &id); }
}
- (instancetype)initWithFrame:(NSRect)frame {
  if ((self = [super initWithFrame:frame])) { [self resetScene]; [NSTimer scheduledTimerWithTimeInterval:1.0 / 60.0 target:self selector:@selector(tick:) userInfo:nil repeats:YES]; }
  return self;
}
- (void)dealloc { if (context_) dq_context_destroy(context_); [super dealloc]; }
- (void)tick:(NSTimer*)timer { (void)timer; if (!paused_) dq_step(context_, 1.0f / 60.0f); [self setNeedsDisplay:YES]; }
- (NSPoint)point:(dq_vec3)p { NSRect r = self.bounds; return NSMakePoint(NSMidX(r) + p.x * kScale, r.size.height * 0.72 - p.y * kScale); }
- (void)drawRect:(NSRect)dirty {
  (void)dirty;
  [[NSColor colorWithRed:0.07 green:0.09 blue:0.13 alpha:1] setFill]; NSRectFill(self.bounds);
  [[NSColor colorWithWhite:0.22 alpha:1] setStroke];
  for (CGFloat y = 0; y < self.bounds.size.height; y += 45) { NSBezierPath* line = [NSBezierPath bezierPath]; [line moveToPoint:NSMakePoint(0,y)]; [line lineToPoint:NSMakePoint(self.bounds.size.width,y)]; [line stroke]; }
  dq_particle_view view{};
  if (dq_particles_acquire(context_, &view) != DQ_OK) return;
  [[NSColor colorWithRed:0.35 green:0.75 blue:1 alpha:1] setStroke];
  for (int i=1;i<kCount;++i) { NSBezierPath* l=[NSBezierPath bezierPath]; [l moveToPoint:[self point:view.positions[ids_[i-1]]]]; [l lineToPoint:[self point:view.positions[ids_[i]]]]; [l setLineWidth:2]; [l stroke]; }
  for (uint32_t i=0;i<view.count;++i) { NSPoint p=[self point:view.positions[i]]; NSColor* color = i == 0 ? [NSColor orangeColor] : [NSColor whiteColor]; [color setFill]; NSRectFill(NSMakeRect(p.x-(i==0?7:5),p.y-(i==0?7:5),i==0?14:10,i==0?14:10)); }
  dq_particles_release(context_, view.generation);
  NSDictionary* attrs=@{NSFontAttributeName:[NSFont systemFontOfSize:16], NSForegroundColorAttributeName:[NSColor whiteColor]};
  [[NSString stringWithFormat:@"Dynquora XPBD | %@ | Space: pausa | N: paso | R: reiniciar", paused_ ? @"pausado" : @"ejecutando"] drawAtPoint:NSMakePoint(20,self.bounds.size.height-30) withAttributes:attrs];
}
- (void)keyDown:(NSEvent*)event { NSString* k=event.charactersIgnoringModifiers.lowercaseString; if ([k isEqualToString:@" "]) paused_=!paused_; else if ([k isEqualToString:@"n"]) { dq_step(context_,1.0f/60.0f); } else if ([k isEqualToString:@"r"]) [self resetScene]; [self setNeedsDisplay:YES]; }
@end

int main() {
  [NSApplication sharedApplication];
  NSRect rect=NSMakeRect(0,0,1280,720);
  NSWindow* window=[[NSWindow alloc] initWithContentRect:rect styleMask:(NSWindowStyleMaskTitled|NSWindowStyleMaskClosable|NSWindowStyleMaskResizable) backing:NSBackingStoreBuffered defer:NO];
  [window setTitle:@"Dynquora Physics — XPBD viewer"]; [window setContentView:[[DQView alloc] initWithFrame:rect]]; [window makeKeyAndOrderFront:nil]; [NSApp activateIgnoringOtherApps:YES]; [NSApp run];
}
