if (self.CavalryLogger) { CavalryLogger.start_js(["KQdP9"]); }

__d("XComposerActorChangeController",["XController"],(function a(b,c,d,e,f,g){c.__markCompiled&&c.__markCompiled();f.exports=c("XController").create("\/composer\/actor\/change\/",{composer_id:{type:"String",required:true},entry_point:{type:"Enum",required:true,enumType:1},from_actor_id:{type:"Int",required:true},target_id:{type:"Int",required:true}});}),null);
__d('ComposerXActorSelector.react',['fbt','ActorSelector.react','ActorSelectorNuxTypes','ActorURI','Arbiter','AsyncRequest','ReactComposerPropsAndStoreBasedStateMixin','ReactComposerStore','React','emptyFunction','XActorSelectorNuxSeenWriteController','XComposerActorChangeController'],(function a(b,c,d,e,f,g,h){if(c.__markCompiled)c.__markCompiled();var i=c('React').PropTypes,j=c('React').createClass({displayName:'ComposerXActorSelector',mixins:[c('ReactComposerPropsAndStoreBasedStateMixin')(c('ReactComposerStore'))],propTypes:{actorIDs:i.array.isRequired,composerID:i.string.isRequired,entryPoint:i.string.isRequired,loading:i.bool,nuxEnabled:i.bool,nuxHoverContext:i.object,onChange:i.func,pageTimelineNuxTipID:i.string,selectedActorID:i.string.isRequired,settingsURI:i.string,targetID:i.string.isRequired},statics:{calculateState:function k(l){return {disabled:c('ReactComposerStore').isPosting(l)};}},getInitialState:function k(){return {loading:false};},componentWillMount:function k(){this._arbiterToken=c('Arbiter').subscribe('ComposerXPages/composePostWithActor',function(l,m){if(m.actorID){if(this.props.onChange){this.props.onChange(m.actorID);return;}this._updateCurrentActor(m.actorID,function(){if(m.callback)m.callback();});}}.bind(this));},componentWillUnmount:function k(){c('Arbiter').unsubscribe(this._arbiterToken);},render:function k(){return (c('React').createElement(c('ActorSelector.react'),{actorIDs:this.props.actorIDs,className:this.props.className,disabled:this.state.disabled,loading:this.props.loading||this.state.loading,nuxBody:this._getNUXBody(),nuxEnabled:this.props.nuxEnabled,nuxHoverContext:this.props.nuxHoverContext,onChange:this._onChange,onCompleteNux:this._onCompleteNux,pageTimelineNuxTipID:this.props.pageTimelineNuxTipID,ref:'selector',selectedActorID:this.props.selectedActorID,settingsURI:this.props.settingsURI,tooltipConstructor:this._getTooltipForActorName}));},_getNUXBody:function k(){return h._("Post as yourself or as one of the Pages you manage.");},_getTooltipForActorName:function k(l){return h._("Posting as {actorName}",[h.param('actorName',l)]);},_onChange:function k(l){if(this.props.onChange){this.props.onChange(l.value);}else this._updateCurrentActor(l.value);},_updateCurrentActor:function k(l,m){m=m||c('emptyFunction');if(this.props.selectedActorID===l){m();return;}this.setState({loading:true});var n=c('ActorURI').create(c('XComposerActorChangeController').getURIBuilder().setString('composer_id',this.props.composerID).setEnum('entry_point',this.props.entryPoint).setInt('from_actor_id',this.props.selectedActorID).setInt('target_id',this.props.targetID).getURI(),l),o=function(){this.setState({loading:false});}.bind(this);new (c('AsyncRequest'))().setURI(n).setErrorHandler(o).setServerDialogCancelHandler(o).setFinallyHandler(m).send();},_onCompleteNux:function k(){var l=c('XActorSelectorNuxSeenWriteController').getURIBuilder().setEnum('nux_type',c('ActorSelectorNuxTypes').COMPOSER).getURI();new (c('AsyncRequest'))().setURI(l).send();}});f.exports=j;}),null);
__d("XReactComposerActorChangeController",["XController"],(function a(b,c,d,e,f,g){c.__markCompiled&&c.__markCompiled();f.exports=c("XController").create("\/react_composer\/actor\/change\/",{composer_id:{type:"String",required:true},composer_type:{type:"Enum",required:true,enumType:1},target_id:{type:"String",required:true},group_sell_composer_sell_availability:{type:"Enum",enumType:1}});}),null);
__d('ReactComposerActorSelectorContainer.react',['cx','ReactComposerContextMixin','ComposerXActorSelector.react','ActorURI','AsyncRequest','React','XReactComposerActorChangeController'],(function a(b,c,d,e,f,g,h){if(c.__markCompiled)c.__markCompiled();var i=c('React').createClass({displayName:'ReactComposerActorSelectorContainer',mixins:[c('ReactComposerContextMixin')],getInitialState:function j(){return {showSpinner:false};},render:function j(){var k=babelHelpers['extends']({},this.props,{composerID:this.context.composerID});return (c('React').createElement(c('ComposerXActorSelector.react'),babelHelpers['extends']({},k,{className:"_4w4v",loading:this.state.showSpinner,onChange:this._onActorChange})));},_onActorChange:function j(k){this.setState({showSpinner:true});var l=c('ActorURI').create(c('XReactComposerActorChangeController').getURIBuilder().setString('composer_id',this.context.composerID).setEnum('composer_type',this.context.composerType).setString('target_id',this.context.targetID).getURI(),k);new (c('AsyncRequest'))().setURI(l).send();}});f.exports=i;}),null);