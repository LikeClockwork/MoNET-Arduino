if (self.CavalryLogger) { CavalryLogger.start_js(["FCDEV"]); }

__d('createContentStateForTextWithMessengerEmoji',['ContentState','DraftModifier','MessengerSupportedEmoji','insertMessengerEmojiIntoContentState'],(function a(b,c,d,e,f,g){'use strict';if(c.__markCompiled)c.__markCompiled();function h(i){var j=c('ContentState').createFromText('');while(i){var k=c('MessengerSupportedEmoji').findEmoji(i);if(k){var l=i.substr(0,k.index),m=k[0],n=i.substr(k.index+m.length);j=c('DraftModifier').insertText(j,j.getSelectionAfter(),l);j=c('insertMessengerEmojiIntoContentState')(k.emoji,j,j.getSelectionAfter(),j.getFirstBlock().getInlineStyleAt(0));i=n;}else break;}if(i)j=c('DraftModifier').insertText(j,j.getSelectionAfter(),i);return j;}f.exports=h;}),null);
__d('MessengerTextWithEmojiInput.react',['cx','CompositeDraftDecorator','ContentState','DraftEditor.react','EditorState','MercuryConfig','MessengerEmojiSpan.react','React','XUICloseButton.react','createContentStateForTextWithMessengerEmoji','getEntityMatcher','getVisibleValueForContentState','getVisibleValueForContentStateWithMessengerEmoji','handleBeforeInputForMessengerEmoji','joinClasses'],(function a(b,c,d,e,f,g,h){'use strict';var i,j;if(c.__markCompiled)c.__markCompiled();var k=c('React').PropTypes;i=babelHelpers.inherits(l,c('React').PureComponent);j=i&&i.prototype;function l(){var m,n;for(var o=arguments.length,p=Array(o),q=0;q<o;q++)p[q]=arguments[q];return n=(m=j.constructor).call.apply(m,[this].concat(p)),this.$MessengerTextWithEmojiInput1=function(r){return c('MercuryConfig').MessengerNewEmojiGK?c('EditorState').createWithContent(c('createContentStateForTextWithMessengerEmoji')(r||''),this.$MessengerTextWithEmojiInput2()):c('EditorState').createWithContent(c('ContentState').createFromText(r?r:''));}.bind(this),this.$MessengerTextWithEmojiInput5=function(r){if(r!==this.state.editorState){var s=!r.getCurrentContent().hasText();this.setState({editorState:r,isEmpty:s});this.props.onChange&&this.props.onChange(s);}}.bind(this),this.$MessengerTextWithEmojiInput2=function(){return new (c('CompositeDraftDecorator'))([{strategy:c('getEntityMatcher')(function(r){return r.getType()==='EMOJI';}),component:c('MessengerEmojiSpan.react')}]);},this.getValue=function(){var r=this.state.editorState.getCurrentContent();return c('MercuryConfig').MessengerNewEmojiGK?c('getVisibleValueForContentStateWithMessengerEmoji')(r):c('getVisibleValueForContentState')(r);}.bind(this),this.focusInput=function(){this.refs.input.focus();this.$MessengerTextWithEmojiInput5(c('EditorState').moveFocusToEnd(this.state.editorState));}.bind(this),this.$MessengerTextWithEmojiInput3=function(r){if(this.props.maxLength&&this.props.maxLength<=this.getValue().length)return 'handled';if(!c('MercuryConfig').MessengerNewEmojiGK)return 'not-handled';var s=c('handleBeforeInputForMessengerEmoji')(this.state.editorState,r);if(s===this.state.editorState)return 'not-handled';this.setState({editorState:s});return 'handled';}.bind(this),this.$MessengerTextWithEmojiInput4=function(r){this.props.onReturn&&this.props.onReturn(r);return 'handled';}.bind(this),this.$MessengerTextWithEmojiInput6=function(){this.setState({editorState:this.$MessengerTextWithEmojiInput1('')});}.bind(this),this.state={editorState:this.$MessengerTextWithEmojiInput1(this.props.initialValue),isEmpty:!this.props.initialValue||this.props.initialValue.length===0},n;}l.prototype.componentDidMount=function(){this.focusInput();};l.prototype.render=function(){return c('React').createElement('div',{className:c('joinClasses')(this.props.className,"_30e7")},c('React').createElement('div',{className:"_5j5f _3oh-"},c('React').createElement(c('DraftEditor.react'),{editorState:this.state.editorState,handleBeforeInput:this.$MessengerTextWithEmojiInput3,handleReturn:this.$MessengerTextWithEmojiInput4,onChange:this.$MessengerTextWithEmojiInput5,placeholder:this.props.placeholder,ref:'input',spellCheck:false,stripPastedStyles:true,textAlignment:'left'})),c('React').createElement(c('XUICloseButton.react'),{size:'small',onMouseDown:this.$MessengerTextWithEmojiInput6,className:(this.state.isEmpty?"hidden_elem":'')+(' '+"_5j5l")}));};l.propTypes={onReturn:k.func,maxLength:k.number,onChange:k.func,placeholder:k.string,initialValue:k.string};f.exports=l;}),null);
__d("XGroupSideConversationCreateController",["XController"],(function a(b,c,d,e,f,g){c.__markCompiled&&c.__markCompiled();f.exports=c("XController").create("\/groups\/side_conversation\/",{post_id:{type:"Int"}});}),null);